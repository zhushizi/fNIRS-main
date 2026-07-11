# fNIRS 双接收源五波长上位机软件

本仓库现在只保留 **上位机软件部分**，并已切换到 **双接收源 + 五波长** 的 26B 串口协议。
软件接收 `S1_D1`（长距 3.0 cm）与 `S1_D2`（短距 1.0 cm）两个接收源、五个光源波长（850 / 810 / 770 / 730 / 700 nm）的光强数据，完成采集、配对、短距回归校正（SSR）、广义 MBLL 与 rSO₂ 估算，输出 **HbO / HbR / Cyt** 三色团浓度变化以及 **rSO₂**。

> 数据链路：安卓 App 独占 UART 启停，把设备串口字节经 **TCP/JSON** 转发给 PC 解码端（PC 作为 TCP Server 被动接收）。PC 端在采集中并行做在线分析并把结果回传安卓绘图，采集结束后再做一次离线终算落盘。

## 当前协议摘要

- **波特率**：`115200`
- **帧头**：`0x55 0xAA`
- **固定帧长**：`0x1A`（26 字节）
- **帧类型**：
  - `0x01`：指令帧（上位机 -> 下位机）
  - `0x02`：数据帧（下位机 -> 上位机）
  - `0x03`：应答帧（双向 ACK）
- **payload**：`21B`
- **校验**：从长度字节到 payload 末尾的累加和低字节
- **当前固件不回 ACK**：`protocol.send_frame_with_ack` 暂时跳过 ACK 判断（由安卓侧控制 UART）

## 当前几何与算法假设

- **物理通道**：双接收源
  - `S1_D1`：源探距离默认 `3.0 cm`（长距，配置在 `software/config.py`）
  - `S1_D2`：源探距离默认 `1.0 cm`（短距，用于短距回归校正）
- **波长编码**（数据帧 payload byte0 = `Wavelength`）：
  - `0x00`：未点亮（OFF，配对前丢弃）
  - `0x01`：850nm
  - `0x02`：810nm
  - `0x03`：770nm
  - `0x04`：730nm
  - `0x05`：700nm
- **接收源编码**（数据帧 payload byte1 = `DetectorId`）：
  - `0x01`：`S1_D1`
  - `0x02`：`S1_D2`
- **采样值**：payload `byte2:6`，4 字节大端无符号
- **采集通道编码**（数据帧 payload byte6 = `ChannelId`，与接收源/波长正交）：
  - `0x01`：通道1
  - `0x02`：通道2
  - `0x00`/缺省：归入通道1
  - 每个采集通道内部各有完整的双接收源×五波长，反演链路在每个通道内各跑一套；回传安卓时用 `channel` 字段区分
- **反演**：分步广义 MBLL 先解 HbO/HbR，再从残差投影估 Cyt（oxCCO，UCL-NIR 差分消光系数）
- **rSO₂**：基于 30–60s 基线段假设（基线 HbT≈80μM、rSO₂≈65%）把 ΔHbO/ΔHbR 转成血氧饱和度
- **输出通道**：由 `config.OUTPUT_CHANNEL` 选择，可选 `S1_D1` / `S1_D2` / `S1_D1_ssr`，默认 `S1_D1_ssr`（短距回归校正后的长距通道）

## 快速开始

1. 安装依赖

```bash
cd software
pip install -r requirements.txt
```

2. 配置参数

编辑 `software/config.py`：

- `SERIAL_PORT`（仅旧串口脚本使用）
- `BAUD_RATE`
- `HOST_TCP_DEFAULT_PORT`（安卓 TCP 连接端口，默认 `9000`）
- `DETECTOR_CHANNELS` / `WAVELENGTH_CHANNELS`（几何与波长表）
- `OUTPUT_CHANNEL`（输出通道）

3. 运行方式（当前主链路）

```bash
cd software
# 启动 TCP 解码端，等待安卓连接；采集 + 在线回传 + 离线终算
python fNIRS_processing.py

# 同时在 PC 端镜像绘制发往安卓的曲线
python fNIRS_processing.py --live_plot

# 打印 TCP 收发日志
python fNIRS_processing.py --tcp_debug
```

## 输出文件

每次采集在 `software/result_table/<时间戳>/` 下生成一组文件。**采集通道相关的文件按通道各写一份**（后缀 `_ch1` / `_ch2`），原始数据合写一份并带 `ChannelId` 列：

- `all_groups.csv`
  - 原始采集数据（所有通道合写一份）
  - 列：`Time (s), ChannelId, DetectorId, Channel, Wavelength, Value`
- `interleaved_output_ch{n}.csv`
  - 该通道 2 接收源 × 5 波长配对后的光强宽表
  - 列：`Time (s), S1_D1_850, S1_D1_810, ..., S1_D2_700`
- `processed_output_ch{n}.csv`
  - 该通道广义 MBLL + SSR 输出（列前缀取自 `OUTPUT_CHANNEL`）
  - 列风格：`Time, S1_D1_ssr_hbo, S1_D1_ssr_hbr, S1_D1_ssr_cyt`
- `android_live_output_ch{n}.csv`
  - 该通道在线分析过程中实际回传安卓的曲线副本

> 只出现一个通道时就只生成该通道那一套文件（例如仅通道1 时只有 `*_ch1.csv`）。

## 主要文件

- `software/config.py`：串口/TCP 参数、几何与五波长表、消光系数、输出通道
- `software/protocol.py`：26B 协议组帧 / 拆帧 / 校验
- `software/fNIRS_processing.py`：主入口（解析参数后调用 `run_pipeline`）
- `software/fnirs_pipeline/`：采集 → 预处理 → 配对 → MBLL/SSR 的离线/在线管线
- `software/online_android/`：安卓 TCP 桥接、在线分析后台线程、rSO₂、回传与绘图

更详细的数据链路说明见 `software/fNIRS_processing_pipeline.md` 与 `software/README.md`。

## 历史 / 单通道脚本

下列脚本来自旧的单通道双波长（660/940）版本，部分仍走本地串口，主链路已不再使用，仅作参考或离线回放：
`adc_live.py`、`adc_animation.py`、`mBLL_animation.py`、`visualizer.py`（+ `index.html`）、`hbo_hbr_live.py`、`data_analysis.py`、`nirs_viewer.py`、`rso2_from_processed.py`。
