# 单通道 fNIRS 处理链路说明

本文档说明当前 `software/fNIRS_processing.py` 的单通道处理链路。  
当前版本已不再使用旧的 64B 多组协议，而是采用 **26B 带帧头/校验/ACK** 的单通道协议。

---

## 1. 总览

```text
26B 串口数据帧
  -> 帧头同步 / 长度校验 / checksum 校验
  -> 回 0x03 ACK
  -> capture_data -> all_groups.csv
  -> threshold_filter
  -> butter_lowpass_filter
  -> sliding_window_rms
  -> interleave_mode_blocks
  -> interleaved_output.csv
  -> process_csv_dataset
      -> 660/940 配对
      -> intensities_to_od_changes
      -> smart_bandpass
      -> mbll
      -> cbsi
  -> processed_output.csv
```

---

## 2. 串口协议层

### 2.1 串口参数

- 波特率：`115200`
- 帧头：`0x55 0xAA`
- 固定帧长：`26B`
- 类型：
  - `0x01` 指令帧
  - `0x02` 数据帧
  - `0x03` 应答帧

### 2.2 校验规则

- 校验范围：从 **长度字节** 到 **payload 最后一个字节**
- 校验方式：**累加和低字节**

### 2.3 ACK / 重发

- ACK 超时：`10ms`
- 最多重发：`2` 次
- 上位机收到 `0x02` 数据帧后会立即回 `0x03` ACK

### 2.4 指令帧 `0x01`

payload 共 21 字节，目前使用：

| 字节 | 含义 |
|------|------|
| 0 | 启停：`0x00` 停止，`0x01` 启动 |
| 1 | 光强高字节 |
| 2 | 光强低字节 |
| 3-20 | 保留，当前填 0 |

默认光强：`0x012C`（300）

### 2.5 数据帧 `0x02`

payload 共 21 字节，目前使用：

| 字节 | 含义 |
|------|------|
| 0 | 波长编号：`0x00=未点亮`，`0x01=940nm`，`0x02=660nm` |
| 1 | 传感器编号：当前固定 `0x00`（单个传感器模块 `S1_D1`） |
| 2 | 采集值低字节 |
| 3 | 采集值高字节 |
| 4-19 | 保留 |
| 20 | 保留位，当前固定 `0x00` |

单个数据帧只携带 **一个通道、一个波长下的一次采样值**。

说明：

- 当前上位机里，`payload byte0` 的**波长编号**同时承担了原先“emitter-state”的语义（含未点亮 `0x00`）。
- **`0x00` 行在 `run_pipeline` 读入 `all_groups.csv` 后即丢弃**，不参与 RMS 切段与 660/940 交错配对。
- 分段、配对、MBLL 组织仍只依赖 CSV 中的 `Wavelength` 列（有效值仅为 `1` 与 `2`）。

---

## 3. 原始采集层

### 3.1 `capture_data`

`capture_data()` 的职责：

1. 打开串口
2. 发送启动命令帧
3. 按帧头同步读取 26B 数据帧
4. 校验通过后回 ACK
5. 将数据写入 `all_groups.csv`
6. 采集结束后发送停止命令帧

### 3.2 `all_groups.csv` 结构

```text
Time (s),SensorId,S1_D1,Wavelength
```

字段说明：

- `Time (s)`：接收该帧时的相对时间
- `SensorId`：传感器编号，当前固定为 `0`
- `S1_D1`：当前单通道采样值
- `Wavelength`：波长编号（`0=未点亮`，`1=940`，`2=660`；流水线中会去掉 `0`）

这里不再有 `Short / Long1 / Long2`，因为当前几何只有 **单物理通道 `S1_D1`**。

---

## 4. 预处理层

输入：`all_groups.csv`  
输出：`interleaved_output.csv`

### 4.1 阈值滤波 `threshold_filter`

- 针对 `S1_D1` 做阈值过滤
- 越界样本先置空，再插值补齐

### 4.2 低通滤波 `butter_lowpass_filter`

- 对 `S1_D1` 沿时间轴做低通
- 数据太短时自动跳过，避免 `filtfilt` 报错

### 4.3 分段 RMS `sliding_window_rms`

- 依据 `Wavelength` 变化切段
- 每段把 `S1_D1` 用 RMS 代表值回填整段

目的仍然是：

- 用一个稳定值代表同一波长时隙
- 方便后续 660 / 940 配对

### 4.4 模式交错 `interleave_mode_blocks`

- 将相邻的两个波长块成对处理
- 自动识别哪一块对应 660，哪一块对应 940
- 生成一行一对的输出

### 4.5 `interleaved_output.csv` 结构

```text
Time (s),S1_D1_660,S1_D1_940
```

每一行表示一组已经对齐好的双波长样本。

---

## 5. 后处理层

入口函数：`process_csv_dataset`

输入：`interleaved_output.csv`  
输出：`processed_output.csv`

### 5.1 双波长配对

当前已不需要旧版本那种：

- 8 组
- 每组 3 路
- 共 48 个样本量

现在直接从 `interleaved_output.csv` 中取：

- `S1_D1_660`
- `S1_D1_940`

组合为形状 `(2, N)` 的样本矩阵。

### 5.2 `build_channel_info`

当前几何固定为：

- 通道名：`S1_D1`
- 距离：`3.0 cm`（来自 `config.py`）
- 波长：`660 / 940`

### 5.3 OD 转换

```python
delta_od = nsp.intensities_to_od_changes(samples)
```

### 5.4 带通滤波 `smart_bandpass`

- 默认带通：`0.05 ~ 0.1 Hz`
- 样本太短时自动跳过滤波

### 5.5 MBLL 与 CBSI

使用单通道的：

- `channel_names = [S1_D1, S1_D1]`
- `wavelengths = [660, 940]`
- `distances = [3.0, 3.0]`

最终输出：

- `S1_D1_hbo`
- `S1_D1_hbr`

### 5.6 `processed_output.csv` 结构

```text
Time,S1_D1_hbo,S1_D1_hbr
```

这仍然保持旧版的输出风格：  
第一列时间，后面采用 `{ChannelName}_{Type}` 形式。

---

## 6. 关键配置

位置：`software/config.py`

重点参数：

- `SERIAL_PORT`
- `BAUD_RATE = 115200`
- `ACK_TIMEOUT_SECONDS = 0.01`
- `MAX_RETRIES = 2`
- `DEFAULT_INTENSITY_MA = 300`
- `SOURCE_DETECTOR_DISTANCE_CM = 3.0`

---

## 7. 与旧版的核心差异

| 项目 | 旧版 | 当前版 |
|------|------|--------|
| 协议 | 64B 裸帧 | 26B 带帧头/校验/ACK |
| 通道数 | 8组 x 3 路 | 1 路 `S1_D1` |
| 波长组织 | 由多组 emitter 状态切段 | 单通道按 `Wavelength` 的 660/940 配对 |
| 原始 CSV | `G0...G7` 宽表 | `Time,S1_D1,Wavelength` |
| 处理中间表 | 33 列宽表 | 单通道双波长配对表 |
| 几何 | 短距/长距混合 | 单距离 `3.0 cm` |

---

## 8. 运行建议

1. 先确认 `config.py` 中串口端口正确
2. 用 `python visualizer.py` 验证控制命令与 ACK
3. 用 `python adc_live.py` 查看实时波形
4. 用 `python fNIRS_processing.py` 完成一次采集和处理

如果 `processed_output.csv` 行数太少，优先检查：

- 是否确实收到了 `0x02` 数据帧
- `Wavelength` 是否在 660 / 940 之间切换
- ACK 是否正常返回
- 原始采样是否足够形成多个配对样本
