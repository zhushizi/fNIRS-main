"""在线会话运行时可变的 BL（基线 rSO2）状态。"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field


@dataclass
class MutableBaseline:
    """线程安全的 BL / HbT 假设，供 set_baseline 在采集中更新。"""

    rso2_pct: float
    hbt_uM: float
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False, compare=False)

    def update(
        self,
        rso2_pct: float,
        hbt_uM: float | None = None,
    ) -> tuple[float, float]:
        with self._lock:
            self.rso2_pct = float(rso2_pct)
            if hbt_uM is not None:
                self.hbt_uM = float(hbt_uM)
            return self.rso2_pct, self.hbt_uM

    def snapshot(self) -> tuple[float, float]:
        with self._lock:
            return self.rso2_pct, self.hbt_uM
