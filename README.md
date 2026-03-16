# fNIRS 上位机软件（Host PC Software）

本仓库为 **fNIRS 上位机部分**：通过串口连接设备、接收 64 字节数据帧，完成采集、预处理与血氧反演（MBLL/CBSI）等流程。不包含下位机固件与硬件设计。

## 功能概览

- **串口通信**：连接 USB 虚拟串口，按 64 字节固定帧接收 8 组 × (Short/Long1/Long2 + Emitter) 数据
- **数据采集**：解析帧并写入 CSV（`all_groups.csv`）
- **预处理**：阈值滤波、低通、分段 RMS、模式交错、时间轴重建
- **后处理**：双波长配对、OD 转换、带通滤波、MBLL、CBSI，输出 HbO/HbR
- **可视化**：实时曲线、静态/交互图表、3D 脑模型等（见 `software/`）

## 快速开始

1. **配置串口**  
   编辑 `software/config.py`，将 `SERIAL_PORT` 设为本机 COM 口（如 Windows 下 `"COM3"`，macOS/Linux 下 `/dev/tty.usbmodem...`）。

2. **安装依赖**  
   ```bash
   cd software
   pip install -r requirements.txt
   ```

3. **采集与处理**  
   - 连接设备后运行：`python fNIRS_processing.py`，按提示采集并生成 `all_groups.csv`、`interleaved_output.csv`、`processed_output.csv`。
   - 仅看实时曲线：`python adc_live.py`。

4. **无设备演示**  
   可使用 `adc_mock_server.py` 等模拟数据（见 `software/README.md`）。

## 目录结构

```
fNIRS-main/
├── software/          # 上位机 Python 代码（串口、处理、可视化）
│   ├── config.py      # 串口与采样参数
│   ├── fNIRS_processing.py
│   ├── adc_live.py
│   ├── visualizer.py
│   └── ...
├── README.md
└── ...
```

协议与处理链路详见 `software/fNIRS_processing_pipeline.md`。
