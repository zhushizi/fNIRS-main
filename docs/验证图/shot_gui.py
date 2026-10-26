# -*- coding: utf-8 -*-
"""离屏回放真实归档，截一张界面图。

    python shot_gui.py [输出.png] [归档名，不带后缀] [时间窗：1 分钟/10 分钟/1 小时/全程]

不给归档名时取最大的那份。
"""
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
from srs_cal.online import (CAL_DIR, DEFAULT_CAL, SPAN_CHOICES, Calibration,
                            OnlineWindow)

app = QtWidgets.QApplication([])
w = OnlineWindow(Calibration.load(CAL_DIR / DEFAULT_CAL))
w.chk_rec.setChecked(False)
w.resize(1180, 1000)
w.ensurePolished()

if len(sys.argv) > 2:
    path = archive.DEFAULT_DIR / (sys.argv[2] + ".srsraw")
else:
    path = max(archive.list_archives(), key=lambda p: p.stat().st_size)
if len(sys.argv) > 3:
    w.cmb_span.setCurrentIndex([n for n, _ in SPAN_CHOICES].index(sys.argv[3]))
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
print("归档", path.name, " TOI 点数", len(w.toi_hist),
      f" 时长 {w.t_hist[-1]:.0f}s" if w.t_hist else "")
print("基线：", w.lbl_bl.text())
print("AUC ：", w.lbl_auc.text(), f"累计 {w.bl.auc.value_min:.2f} %·min")
