# -*- coding: utf-8 -*-
"""SRS 体模标定 —— 新硬件的独立验证环境。

硬件：1 光源 + 2 长距探测器 @ 3.0 / 4.0 cm，
      五波长 670 / 770 / 810 / 850 / 950 nm（950 不进反演，见 hardware.py）。
体模：MH Limited P3137034，12 块均质圆柱，认证于 685/808/830/975 nm。

与现役 MBLL 产品线的关系
------------------------
本包**不 import `config`**。`config.py` 描述的是另一台机器
（850/810/770/730/700、D2=1.0 cm 短距），两边各自演进、互不牵连。

算法则**复用** `fnirs_pipeline.srs` 本体 —— 该模块只依赖 `.mbll_core`，
几何与标定常数全部经 `SrsGeometry` 传参，所以这里跑的就是产品代码，
不是副本。`srs.py` 自身的验证状态见其文件头，另见
`tools/srs_verify_chain.py`（15/15 通过）。

    python -m srs_cal.selftest      # 在 software/ 目录下运行

⚠️ 全部内容基于均匀半无限介质假设。真实头部为分层结构，
   本包不验证分层几何，也**未经临床验证，不可用于诊断**。
"""

from __future__ import annotations

__all__ = ["hardware", "phantom", "forward", "calibrate"]
