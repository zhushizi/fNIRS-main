# fNIRS 单通道上位机软件

本仓库现在只保留 **上位机软件部分**，并已切换到新的 **26B 单通道串口协议**。  
软件通过串口与设备通信，接收单通道 `S1_D1` 的双波长数据（660nm / 940nm），完成采集、配对、MBLL 和 CBSI 处理，并输出 `hbo / hbr`。

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
- **ACK 规则**：10ms 超时，最多重发 2 次

## 当前几何与算法假设

- **物理通道**：单通道 `S1_D1`
- **源探距离**：默认 `3.0 cm`，配置在 `software/config.py`
- **波长编码**（数据帧 payload 首字节）：
  - `0x00`：未点亮（不参与 660/940 配对）
  - `0x01`：940nm
  - `0x02`：660nm
- **术语约定**：
  - 数据帧 `payload byte0` 表示当前样本对应的发射状态（含未点亮）；配对与 MBLL 前会丢弃 `0x00`
  - 离线处理统一按 CSV 中的 `Wavelength` 列分段（仅保留 `0x01` / `0x02` 参与 660/940 成对）
- **输出风格**：`processed_output.csv` 保持 `Time + {ChannelName}_{Type}` 样式

## 快速开始

1. 安装依赖

```bash
cd software
pip install -r requirements.txt
```

2. 配置串口

编辑 `software/config.py`：

- `SERIAL_PORT`
- `BAUD_RATE`
- `SOURCE_DETECTOR_DISTANCE_CM`

3. 运行方式

- **采集 + 处理**：`python fNIRS_processing.py`
- **实时 ADC 曲线**：`python adc_live.py`
- **控制面板**：`python visualizer.py`
- **演示模式控制面板**：`python visualizer.py demo`

## 输出文件

- `all_groups.csv`
  - 原始单通道采集数据
  - 列：`Time (s), SensorId, S1_D1, Wavelength`
- `interleaved_output.csv`
  - 660/940 配对后的中间结果
  - 列：`Time (s), S1_D1_660, S1_D1_940`
- `processed_output.csv`
  - MBLL + CBSI 输出
  - 列风格：`Time, S1_D1_hbo, S1_D1_hbr`

## 主要文件

- `software/config.py`：串口和协议配置
- `software/protocol.py`：26B 协议收发、校验、ACK、重发
- `software/fNIRS_processing.py`：采集、配对、单通道 MBLL
- `software/adc_live.py`：单通道实时 ADC 显示
- `software/visualizer.py`：轻量控制面板后端
- `software/index.html`：轻量控制面板前端

更详细的数据链路说明见 `software/fNIRS_processing_pipeline.md`。
