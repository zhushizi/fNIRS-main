# -*- coding: utf-8 -*-
"""MBLL 解码器 —— 与 SRS 侧完全独立的相对量通路。

**只输出 ΔHbO / ΔHbR。血氧绝对值由 SRS 给出**（2026-10-22 定版）。

隔离边界（docs/双解码器架构.md §三）：
    共享：L2 原始记录、`fnirs_pipeline.mbll_core`（消光表/DPF，纯常数）
    不共享：预处理、配置、标定、输出
"""

from mbll_decoder.decoder import MbllResult, decode   # noqa: F401
