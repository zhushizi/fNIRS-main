# -*- coding: utf-8 -*-
"""离屏回放真实归档，截一张界面图。"""
import os
import sys
import time
from pathlib import Path

#  用真实平台渲染，但不 show() —— 不弹窗打扰用户，又能把字画出来
ROOT = Path(r"D:\WORK_space\Code_WS\code_localized\fNIRS-main")
sys.path.insert(0, str(ROOT / "software"))
sys.stdout.reconfigure(encoding="utf-8")

from PyQt5 import QtWidgets

from srs_cal import archive
from srs_cal.online import CAL_DIR, DEFAULT_CAL, Calibration, OnlineWindow

app = QtWidgets.QApplication([])
w = OnlineWindow(Calibration.load(CAL_DIR / DEFAULT_CAL))
w.chk_rec.setChecked(False)
w.resize(1180, 1000)
w.ensurePolished()

path = max(archive.list_archives(), key=lambda p: p.stat().st_size)
frames = archive.replay(path)
for i in range(0, len(frames), 20):
    w._on_frames(frames[i:i + 20])
w._refresh()
app.processEvents()
time.sleep(0.4)
app.processEvents()

out = Path(sys.argv[1] if len(sys.argv) > 1
           else r"C:\Users\24114\AppData\Local\Temp\claude\shot.png")
w.grab().save(str(out))
print("saved", out)
print("TOI 点数", len(w.toi_hist), " MBLL 周期", w._mbll.stats.n_cycles)
print("MBLL 状态：", w.lbl_mbll.text())
