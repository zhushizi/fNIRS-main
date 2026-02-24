# A Wearable Functional Near-Infrared Spectroscopy (fNIRS) based Brain Interface

<div align="center">
  <img src="https://github.com/user-attachments/assets/e845183c-7722-4132-92f5-48b59a016dfe" alt="Device Overview" width="400"/>
</div>

## Project Overview

This project introduces a low-cost, ergonomic functional Near-Infrared Spectroscopy (fNIRS) device designed for real-time brain activity monitoring. fNIRS is a non-invasive technique that uses near-infrared light to measure changes in blood oxygenation, providing insights into neural activity based on the modified Beer-Lambert Law.

While conventional fNIRS systems are expensive, bulky, and limited to clinical environments, this system is designed to be accessible, accurate, and user-friendly—ideal for education, personal health tracking, and portable research applications.

### System Highlights

- 24 custom sensor modules supporting 24 detectors and 8 dual-wavelength LED emitters (660 nm & 990 nm)
- Custom electrical control unit (ECU) built around a low-power STM32 microcontroller (STM32L476RET6) that interfaces with all sensor modules for synchronized control and data acquistion
- Ergonomic mechanical design for secure, comfortable wear during extended sessions
- Interactive Python-based GUI for live data visualization, serial communication, and module control

### 下位机验证
- 先改 software/config.py 的 SERIAL_PORT 为你电脑上的 COM 口（Windows 如 COM3）。
- 跑 python adc_live.py 能看到 8 组曲线动态变化 = 板子数据链路通了。
### 上位机验证
- 再跑 python fNIRS_processing.py 验证能否采集并生成 all_groups.csv / processed_output.csv。

一句话：先 adc_live.py 验硬件链路，再 fNIRS_processing.py 验处理链路。