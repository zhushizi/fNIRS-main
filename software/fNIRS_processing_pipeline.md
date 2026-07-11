# 双接收源五波长 fNIRS 处理链路说明

本文档说明当前 `software/fNIRS_processing.py` → `fnirs_pipeline` 的处理链路。
当前版本采用 **双接收源（S1_D1 长距 / S1_D2 短距）+ 五波长（850/810/770/730/700 nm）** 几何，
数据经 **安卓 TCP 桥接** 进入 PC 解码端，最终反演 **HbO / HbR / Cyt** 三色团浓度并估算 **rSO₂**。

> 旧版为单通道双波长（660/940）+ 本地串口直连，本文档不再描述该路径，
> 相关脚本（`adc_live.py` 等）仅作历史保留。

---

## 1. 总览

```text
安卓 App（独占 UART 启停）
  -> TCP/JSON 转发设备串口字节（serial_data, base64）
  -> PC 作为 TCP Server 被动接收（HostTcpSerialBridge）
  -> FrameReader 从字节流同步 26B 数据帧（帧头/长度/checksum）
  -> parse_data_frame -> DataSample(detector_code, wavelength_code, value)
  -> capture_data
       ├─ 写 all_groups.csv
       └─ online_session.feed_sample（喂给在线分析缓冲）
  -> prepare_interleaved_dataframe
       -> butter_lowpass_filter（按 Detector/Wavelength 分组）
       -> sliding_window_rms（按段 RMS）
       -> aggregate_wavelength_cycles（凑齐 2×5 成一行）
       -> interleaved_output.csv
  -> process_csv_dataset
       -> compute_all_channel_concentrations
            ├─ intensities_to_od_changes（每个接收源）
            ├─ smart_bandpass（降采样 + 带通）
            ├─ short_separation_regression（短距校正长距 -> *_ssr）
            └─ generalized_mbll（HbO/HbR + 残差投影 Cyt）
       -> select_output_series（取 config.OUTPUT_CHANNEL）
       -> processed_output.csv
       -> summarize_processed_concentrations（+ rSO₂ 摘要）

并行：OnlineAnalysisWorker 每 ~1s 对【全会话】重算一次，
      经 reporter 把整段 HbO/HbR/Cyt/rSO₂ 回传安卓（replace_full_series=true），
      并镜像写 android_live_output.csv。
```

---

## 2. 传输与协议层

### 2.1 安卓 TCP 桥接（`online_android/tcp_bridge.py`）

- PC 监听 `HOST_TCP_LISTEN_HOST:HOST_TCP_DEFAULT_PORT`（默认 `0.0.0.0:9000`），安卓主动连接
- 消息为 `4 字节大端长度 + UTF-8 JSON`，字段：`ver / type / seq / ts_ms / body`
- 关键消息类型：
  - `hello` / `hello_ack`：握手
  - `serial_data`：安卓上行，`body.payload_b64` 为 base64 的设备串口字节
  - `heartbeat`：心跳
  - `bye` / `bye_ack`：安卓请求结束采集（PC 置 `capture_finished`）
  - `live_analysis_batch` / `analysis_result`：PC 下行的分析结果
- `HostTcpSerialBridge` 把收到的串口字节缓存，并暴露 `read()` / `in_waiting`，从而对上层伪装成一个串口对象

### 2.2 26B 帧格式（`config.py` / `protocol.py`）

- 帧头：`0x55 0xAA`，固定帧长：`0x1A`（26B），payload `21B`
- 类型：`0x01` 指令帧 / `0x02` 数据帧 / `0x03` 应答帧
- 校验：从 **长度字节** 累加到 **payload 末尾** 的低字节
- ACK：当前固件不回 ACK，`send_frame_with_ack` 直接返回 True（启停由安卓侧控制）

### 2.3 数据帧 `0x02` payload

| 字节 | 含义 |
|------|------|
| 0 | 波长编号：`0x00=未点亮`，`0x01=850`，`0x02=810`，`0x03=770`，`0x04=730`，`0x05=700` nm |
| 1 | 接收源编号：`0x01=S1_D1`，`0x02=S1_D2` |
| 2-5 | 采集值（4 字节大端无符号） |
| 6 | 采集通道编号：`0x01=通道1`，`0x02=通道2`（`0x00`/缺省归入通道1） |
| 7-20 | 保留 |

单帧只携带 **一个采集通道、一个接收源、一个波长下的一次采样值**。
`Wavelength=0x00`（OFF）行在配对前丢弃，不参与 RMS 切段与多波长配对。
**采集通道（byte6）与接收源、波长正交**：PC 先按通道拆分数据流，每个通道内部各有
完整的双接收源×五波长，整条 配对 → SSR → MBLL → rSO₂ 链路在每个通道内各跑一套，
在线/离线结果都按通道分别输出。

---

## 3. 原始采集层（`fnirs_pipeline/capture.py`）

`capture_data()` 的职责：

1. 打开 TCP 桥（`HostTcpSerialBridge`）并等待安卓连接
2. 创建并启动在线分析会话 `create_online_session`
3. 用 `FrameReader` 按帧头同步读取 26B 数据帧
4. 校验通过后 `parse_data_frame` 解出 `DataSample`
5. `feed_sample` 喂给在线缓冲，并把每帧写入 `all_groups.csv`
6. 收到安卓 `bye`（`capture_finished`）或连接关闭时结束采集

### `all_groups.csv` 结构

```text
Time (s),ChannelId,DetectorId,Channel,Wavelength,Value
```

- `Time (s)`：相对接收时间
- `ChannelId`：采集通道编号（`1=通道1`，`2=通道2`；离线按此列拆分成 `*_ch{n}.csv`）
- `DetectorId`：接收源编号（`1=S1_D1`，`2=S1_D2`）
- `Channel`：接收源名（`S1_D1` / `S1_D2`）
- `Wavelength`：波长编号（`0=OFF`，`1..5` 对应五波长；流水线中去掉 `0`）
- `Value`：4 字节采样值

> 离线终算 `run_pipeline` 读入 `all_groups.csv` 后按 `ChannelId` 分组，对每个通道分别写
> `interleaved_output_ch{n}.csv` / `processed_output_ch{n}.csv`，并各发一条带 `channel` 字段的
> `analysis_result`。在线 `OnlineAnalysisWorker` 经 `ChannelDispatcher` 为每个通道各维护一个
> 处理器，逐通道回传 `live_analysis_batch`（含 `channel`）。

---

## 4. 预处理层（`fnirs_pipeline/preprocessing.py`）

入口：`prepare_interleaved_dataframe`
输入：`all_groups.csv`（或在线缓冲快照）
输出：`interleaved_output.csv`

### 4.1 丢弃 OFF 行

`Wavelength == WAVELENGTH_OFF_CODE(0x00)` 的样本先剔除。

### 4.2 低通滤波 `butter_lowpass_filter`

- 按 `(DetectorId, Wavelength)` 分组分别低通，避免不同 LED/PD 电平互相污染
- 数据太短自动跳过，避免 `filtfilt` 报错

### 4.3 分段 RMS `sliding_window_rms`

- 依据 `(DetectorId, Wavelength)` 连续段切段
- 每段把采样值压成一行 RMS 代表值，时间取段内均值
- 目的：用一个稳定值代表同一接收源/波长时隙，便于后续配对

### 4.4 多波长配对 `aggregate_wavelength_cycles`

- 在时序上每凑齐 `DETECTOR_CHANNELS × WAVELENGTH_CHANNELS`（当前 2×5=10 个键）即输出一行
- 一行包含全部 10 个光强列
- 配对完成后按平均步长重建等间隔时间戳（可从 0 起算）

### `interleaved_output.csv` 结构

```text
Time (s),S1_D1_850,S1_D1_810,S1_D1_770,S1_D1_730,S1_D1_700,S1_D2_850,...,S1_D2_700
```

每行表示一组对齐好的「2 接收源 × 5 波长」光强样本。

---

## 5. 后处理层（`fnirs_pipeline/mbll.py`）

入口：`process_csv_dataset`（离线终算）/ `calculate_concentration_series`（在线）
核心：`compute_all_channel_concentrations`
输入：`interleaved_output.csv`
输出：`processed_output.csv`

### 5.1 OD 转换

对每个接收源，按 `WAVELENGTH_CHANNELS` 顺序堆叠为 `(5, N)` 光强矩阵，
再 `nsp.intensities_to_od_changes` 得到 ΔOD。

### 5.2 带通滤波 `smart_bandpass`

- 默认带通 `BP_LOW_HZ ~ BP_HIGH_HZ`（`0.01 ~ 0.1 Hz`），4 阶 SOS
- 高采样率时先降采样到 `BP_TARGET_FS_HZ`（默认 20 Hz）再滤波，最后插值回原长度
- 样本太短时自动跳过

### 5.3 短距回归校正 SSR `short_separation_regression`

- 取距离最小的接收源（`S1_D2`，短距）作为头皮/全身干扰参考
- 对每个波长，用短距 ΔOD 对长距 ΔOD 做最小二乘回归并扣除
- 生成新通道 `S1_D1_ssr`（校正后的长距）

### 5.4 广义 MBLL `generalized_mbll`

分步反演，返回 `(3, N)`，行顺序 HbO / HbR / Cyt：

1. HbO/HbR 消光系数来自 nirsimple 内置表（`wray`），插值到五个波长
2. pathlength = `DPF(波长, 年龄) × 源探距离`
3. 先用 `pinv(A_hb) @ ΔOD` 解出 HbO/HbR
4. 计算 Hb 拟合残差 `ΔOD - A_hb·ΔC_hb`
5. 把 Cyt 基向量投影到 Hb 零空间，再从残差中解出 Cyt（UCL-NIR 差分消光系数）

### 5.5 输出通道选择 `select_output_series`

- 从全部通道结果中取 `config.OUTPUT_CHANNEL`（默认 `S1_D1_ssr`）
- 去除非有限值后写盘

### `processed_output.csv` 结构

列前缀取自 `OUTPUT_CHANNEL`：

```text
Time,S1_D1_ssr_hbo,S1_D1_ssr_hbr,S1_D1_ssr_cyt
```

### 5.6 rSO₂ 摘要 `summarize_processed_concentrations`

- 输出 HbO/HbR/Cyt 均值
- 若基线窗内有样本，附加 rSO₂（见第 7 节）

---

## 6. 在线分析与回传（`online_android/`）

- `OnlineSampleBuffer`：累积本会话全部样本
- `OnlineAnalysisWorker`：每 `update_interval_seconds`（默认 1s）触发一次
- `AndroidReporter.send_live_batch`：经 TCP 回传 `live_analysis_batch`，
  同时镜像写 `android_live_output.csv`，可选 PC 端 `live_plotter` 绘图
- 采集结束 `stop()` 时做一次终算落盘

由 `config.ONLINE_MODE` 选择两种在线分析路线：

### 6.1 `full_replace`（旧逻辑，`IncrementalBatchBuilder`）

- 每 tick 对**全会话**重跑 `prepare_interleaved → calculate_series`（非因果 `sosfiltfilt`）
- 仅当配对行数增加（或强制）时才重算
- 每次发送【整段】曲线并置 `replace_full_series=true`
- 缺点：计算/传输随时长 O(N²)，历史曲线每 tick 被改写，量纲不固定

### 6.2 `causal_incremental`（路线 B，`IncrementalCausalProcessor`，默认）

- **增量配对**：自维护流式分段 RMS + 凑组，原始样本只折叠一次，不再每 tick 重跑 prepare
- **固定基准 I0**：用基线窗均值算 ΔOD，历史不随新数据漂移
- **因果 IIR 带通**：`sosfilt` + 持久 `zi` 状态替代零相位滤波，过去的滤波值不再变
- 短距回归 β、显示锚定基线、rSO₂ 基线在**热身末端**（`causal_warmup_seconds`，默认 60s）一次性冻结
- 热身期：用全量 `calculate_series` 出临时曲线（`replace_full_series=true`、`baseline_ready=false`）
- 冻结后：整段回填一次（replace），随后**只追加新样本**（`replace_full_series=false`），单 tick 成本 O(新样本)
- 回传前对浓度做显示换算：摩尔 × `LIVE_CONC_SCALE`（默认 1e6 → μM），并按 `LIVE_BASELINE_ANCHOR` 减基线窗均值锚定，`unit` 字段随之置 `"uM"`
- 代价：用分段 RMS 直接代表波长时隙、跳过离线那步非因果 1Hz 低通，故**在线结果与离线 `processed_output.csv` 不再逐点一致**——离线终算仍是权威结果；因果滤波有群延迟（相位滞后）

---

## 7. rSO₂ 估算（`online_android/rso2.py`）

`compute_rso2_series` 在固定基线假设下，由 ΔHbO/ΔHbR 估算 rSO₂(%)：

- 基线时段：`[baseline_start_s, baseline_end_s]`（默认 `30~60s`）
- 基线 HbT：`DEFAULT_BASELINE_HBT_uM`（默认 `80 μM`）
- 基线 rSO₂：`DEFAULT_BASELINE_RSO2_PCT`（默认 `65%`）
- 用基线段内 ΔHbO/ΔHbR 均值做校正，得到绝对 HbO/HbR，
  再 `rSO₂ = 100 × HbO_abs / (HbO_abs + HbR_abs)`
- 基线窗内无样本时返回 None

> 注意：基线 HbT 与 rSO₂ 为假设常量而非实测，因此 rSO₂ 反映 **相对趋势**，不是绝对标定值。

---

## 8. 关键配置

位置：`software/config.py`

```text
SERIAL_PORT / BAUD_RATE            # 旧串口脚本兜底
HOST_TCP_DEFAULT_PORT = 9000       # 安卓 TCP 端口
SAMPLING_RATE_HZ = 250.0
DETECTOR_CHANNELS                  # S1_D1@3.0cm, S1_D2@1.0cm
WAVELENGTH_CHANNELS                # 850/810/770/730/700 nm
OUTPUT_CHANNEL = "S1_D1_ssr"       # 输出通道
BP_LOW_HZ=0.01 / BP_HIGH_HZ=0.1 / BP_ORDER=4 / BP_TARGET_FS_HZ=20.0
MBLL_DEFAULT_AGE = 27
ONLINE_MODE = "causal_incremental" # 在线路线: full_replace | causal_incremental
LIVE_CONC_SCALE = 1e6              # 回传安卓前浓度换算: 1e6 → μM
LIVE_CONC_UNIT = "uM"
LIVE_BASELINE_ANCHOR = True        # 减基线窗均值，曲线从 0 起算
CYT_DIFFERENCE_EXTINCTION          # 五波长 Cyt 差分消光系数（UCL-NIR）
```

`config.py` 在导入时自检：波长数 ≥ 3、code/emitter/detector 不重复、
OFF code 不在表内、Cyt 消光系数覆盖全部波长、`OUTPUT_CHANNEL` 合法。

---

## 9. 与旧版的核心差异

| 项目 | 旧版（单通道） | 当前版（双接收五波长） |
|------|------|--------|
| 数据来源 | 本地串口直连 | 安卓 TCP 桥接（PC 被动收） |
| 通道数 | 1 路 `S1_D1` | 2 路 `S1_D1`(长) + `S1_D2`(短) |
| 波长 | 660 / 940 | 850 / 810 / 770 / 730 / 700 |
| 配对 | 660/940 两块交错 | 2×5=10 键凑齐成一行 |
| 反演产物 | HbO / HbR | HbO / HbR / Cyt + rSO₂ |
| 头皮干扰 | 无 | 短距回归校正（SSR -> `*_ssr`） |
| 采样值 | 2 字节 | 4 字节大端 |
| 在线分析 | 无 | 后台 worker 全量回传安卓 |
| 原始 CSV | `Time,S1_D1,Wavelength` | `Time,DetectorId,Channel,Wavelength,Value` |

---

## 10. 运行建议

1. 确认 `config.py` 中 `HOST_TCP_DEFAULT_PORT`、`DETECTOR_CHANNELS`、`OUTPUT_CHANNEL` 正确
2. 先启动 `python fNIRS_processing.py`，再让安卓连接并发送 `serial_data`
3. 需要 PC 端实时观察时加 `--live_plot`，排查协议时加 `--tcp_debug`
4. 结果在 `software/result_table/<时间戳>/` 下

如果 `processed_output.csv` 行数太少，优先检查：

- 是否确实收到了 `0x02` 数据帧（看采集日志）
- `Wavelength` 是否在 1..5 之间正常轮转、`DetectorId` 是否覆盖 1 和 2
- 是否能凑齐完整的 2×5 配对周期
- 基线窗（默认 30~60s）内是否有样本（影响 rSO₂）
