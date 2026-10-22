# -*- coding: utf-8 -*-
"""双解码器实施进度图。

状态全部查自真实仓库：git diff --numstat / wc -l / 自检输出 /
docs/双解码器架构.md 的落地顺序表与风险表。不估计。
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

ROOT = Path(r"D:\WORK_space\Code_WS\code_localized\fNIRS-main")

SURF, INK, INK2, INK3 = "#fcfcfb", "#0b0b0b", "#52514e", "#8b8a84"
OK, BAD, NEU, WARN, SKIP = "#1baf7a", "#eb6834", "#2a78d6", "#c9870f", "#9b9a95"
plt.rcParams.update({
    "font.sans-serif": ["Microsoft YaHei", "SimHei", "DengXian", "sans-serif"],
    "axes.unicode_minus": False, "figure.facecolor": SURF, "text.color": INK})

W, H = 100.0, 100.0
fig = plt.figure(figsize=(15.0, 15.0))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W); ax.set_ylim(0, H); ax.axis("off")

DONE, PART, TODO, SKIPPED = "done", "part", "todo", "skip"
COL = {DONE: OK, PART: WARN, TODO: BAD, SKIPPED: SKIP}
MARK = {DONE: "√", PART: "●", TODO: "○", SKIPPED: "—"}


def chip(x, y, text, col, w=None, fs=9.0):
    ax.text(x, y, text, fontsize=fs, color="white", fontweight="bold",
            ha="center", va="center",
            bbox=dict(boxstyle="round,pad=0.34", facecolor=col,
                      edgecolor="none"))


def panel(x, y_top, w, h, title, sub=""):
    ax.add_patch(FancyBboxPatch((x, y_top - h), w, h,
                                boxstyle="round,pad=0.3", linewidth=1.3,
                                edgecolor="#dedcd6", facecolor="#ffffff",
                                clip_on=False))
    ax.text(x + 1.4, y_top - 1.6, title, fontsize=12.5, fontweight="bold",
            va="top")
    if sub:
        ax.text(x + w - 1.4, y_top - 1.9, sub, fontsize=9, color=INK3,
                ha="right", va="top")


ax.text(2.5, H - 1.6, "双解码器实施进度　2026-10-22", fontsize=17,
        fontweight="bold", va="top")
ax.text(97.5, H - 2.0, "第 1~8 条 + L4 并排显示已完成（第 5 条跳过）　"
        "剩 3 条：1 条待你决策 + 2 条实验",
        fontsize=10.5, color=INK3, ha="right", va="top")

# ====================================================== ① 落地顺序
panel(2, H - 6.0, 96, 33.5, "① 落地顺序（docs §六）",
      "自检 139/139 通过（t1~t16）")

ITEMS = [
    ("1", "L1 加 t_host + cycle_id", DONE,
     "解除 R2。实测周期 1521.53 ms，抖动 MAD 0.04 ms"),
    ("2", "L2 在线原始归档", DONE,
     "archive.py 262 行。▲ 缺口：acquire.py 尚未接入"),
    ("3", "稳健聚合代替均值", DONE,
     "实测选型后用 MAD 剔除+均值，不是中位数"),
    ("4", "补回浓度 / 光谱残差 / 波长集校验", DONE,
     "解除 R8。物理判据在真实数据上抓到已知坏段"),
    ("5", "采集帧数 50 → 10~15", SKIPPED,
     "跳过。事后证明方向是反的：实测平均到 ~80 周期仍在降噪"),
    ("6", "L2.5 质量层", DONE,
     "quality.py 313 行。稳定段 0% 误报，坏段 14/14 检出"),
    ("7", "SRS 降噪（改版）", DONE,
     "原「去趋势」否决。改插值扣暗 + N 10→20，合计 2.24×"),
    ("8", "新建 mbll_decoder/", DONE,
     "带通 0.1→0.05 Hz。噪声底 ΔHbO 0.124 µM"),
    ("L4", "一个界面并排看 TOI + ΔHbO/ΔHbR", DONE,
     "live.py 攒缓冲整段重算。界面==命令行，实测逐点差 0"),
    ("9", "档位策略定版", TODO,
     "▲ 唯一待你决策项：共用档位 / 交替分时采（R3）"),
    ("10", "补满 12 块 × 3 次重标", TODO,
     "解决 R7：v5 标定只覆盖组织范围下缘"),
    ("11", "k/b 对标（阻断实验）", TODO,
     "解决 R5：绝对值成立的前提，现在还不成立"),
]
y = H - 11.0
for num, name, st, note in ITEMS:
    chip(6.0, y, f"{num}", COL[st], fs=9.5)
    ax.text(9.5, y, MARK[st], fontsize=11, color=COL[st], va="center",
            fontweight="bold")
    ax.text(12.0, y, name, fontsize=10.2, va="center",
            color=INK if st != SKIPPED else INK3,
            fontweight="bold" if st == TODO else "normal")
    ax.text(45.0, y, note, fontsize=9.0, va="center",
            color=BAD if st == TODO else INK2)
    y -= 2.35

# ====================================================== ② 分层
panel(2, H - 42.0, 46, 24.0, "② 分层状态")
LAYERS = [
    ("L0", "设备", SKIPPED, "厂家，改不了"),
    ("L1", "协议层", DONE, "t_host + cycle_id 已加"),
    ("L2", "原始归档", PART, "online 已接，acquire 未接"),
    ("L2.5", "质量层", DONE, "五类标志，阈值全来自实测"),
    ("L3A", "SRS 解码器", PART, "算法齐，k/b 未标定"),
    ("L3B", "MBLL 解码器", DONE, "本轮新建，负对照已验"),
    ("L4", "展示 / 导出", DONE, "三格并排，本轮补齐"),
]
y = H - 47.6
for code, name, st, note in LAYERS:
    ax.text(5.5, y, MARK[st], fontsize=11.5, color=COL[st], va="center",
            fontweight="bold")
    ax.text(8.5, y, code, fontsize=10.5, va="center", fontweight="bold",
            color=COL[st])
    ax.text(15.5, y, name, fontsize=10.2, va="center", color=INK)
    ax.text(26.0, y, note, fontsize=9.0, va="center", color=INK2)
    y -= 2.4

# ====================================================== ③ 风险
panel(52, H - 42.0, 46, 24.0, "③ 风险（docs §一）")
RISKS = [
    ("R1", "两条链无自检余量", PART,
     "物理判据已补；残差类仍恒为零"),
    ("R2", "采集不记录时间戳", DONE, "已解除"),
    ("R3", "两条链档位需求相反", TODO, "= 第 9 条，待决策"),
    ("R4", "记录长度未定版", DONE, "已解除"),
    ("R5", "绝对量未验证", TODO, "= 第 11 条"),
    ("R6", "670/770「干净」验证面窄", TODO, "待扩验证条件"),
    ("R7", "标定只覆盖范围下缘", TODO, "= 第 10 条"),
    ("R8", "标定文件无波长集校验", DONE, "已解除"),
    ("R9", "在线监护不存数据", DONE, "已解除（见 L2 缺口）"),
    ("R10", "扫描 0.66 周期/秒", SKIPPED, "硬件限制，已在设计中让步"),
]
y = H - 47.3
for code, name, st, note in RISKS:
    ax.text(55.5, y, MARK[st], fontsize=11.5, color=COL[st], va="center",
            fontweight="bold")
    ax.text(58.5, y, code, fontsize=10.5, va="center", fontweight="bold",
            color=COL[st])
    ax.text(64.0, y, name, fontsize=9.8, va="center", color=INK)
    ax.text(82.5, y, note, fontsize=8.8, va="center", color=INK2)
    y -= 2.0

# ====================================================== ④ 本轮产出
panel(2, H - 68.5, 96, 23.0, "④ 本轮产出（git diff --numstat / wc -l）")

NEW = [("srs_cal/archive.py", 262), ("srs_cal/quality.py", 313),
       ("srs_cal/verify_timing.py", 198), ("srs_cal/darksub.py", 129),
       ("srs_cal/robust.py", 93), ("mbll_decoder/（5 个文件）", 603)]
MOD = [("srs_cal/selftest.py", 888, 1), ("srs_cal/online.py", 368, 32),
       ("fnirs_pipeline/srs.py", 174, 11), ("srs_cal/protocol.py", 71, 5),
       ("srs_cal/acquire.py", 26, 7)]

ax.text(5, H - 74.0, f"新建　{sum(n for _, n in NEW)} 行",
        fontsize=10.5, fontweight="bold", color=OK)
y = H - 76.4
for name, n in NEW:
    ax.text(5, y, name, fontsize=9.2, color=INK2)
    ax.text(27, y, f"{n:>4}", fontsize=9.2, color=INK2, ha="right")
    y -= 1.95

ax.text(34, H - 74.0,
        f"改动已有　+{sum(a for _, a, _ in MOD)} / −{sum(b for _, _, b in MOD)} 行",
        fontsize=10.5, fontweight="bold", color=NEU)
y = H - 76.4
for name, a, b in MOD:
    ax.text(34, y, name, fontsize=9.2, color=INK2)
    ax.text(58, y, f"+{a} / −{b}", fontsize=9.2, color=INK2, ha="right")
    y -= 1.95

ax.text(64, H - 74.0, "验证方式", fontsize=10.5, fontweight="bold", color=INK)
for i, s in enumerate([
        "自检 139/139（t1~t16），每条做完即跑",
        "真机验证：COM16 @ 115200，每条都上机",
        "真实归档重放 —— 改了算法能比同一份数据，",
        "   不必让受试者再动一次",
        "验证图在 docs/验证图/，界面截图同处",
        "",
        "自检抓到的设计缺陷：归档文件名碰撞、",
        "质量层误判停用波长、残差阈值灵敏度不足等"]):
    ax.text(64, H - 76.4 - i * 1.95, s, fontsize=9.0,
            color=INK2 if not s.startswith("自检抓到") else INK)

# ====================================================== 脚注
ax.text(2.5, 5.2, "下一步", fontsize=11, fontweight="bold")
ax.text(10, 5.2, "第 9 条「档位策略定版」需要你决策：SRS 与 MBLL 共用一个档位，"
        "还是交替分时采。第 6 条的停用波长监视已能直接看出各档位下加性项 C 是否出现。",
        fontsize=10, color=INK2)
ax.text(2.5, 2.4, "√ 完成　● 部分　○ 待做　— 跳过/不适用　"
        "▲ 缺口　·　所有数字查自真实仓库与真实归档，不是估计",
        fontsize=8.8, color=INK3)

out = ROOT / "docs" / "实施进度图.png"
fig.savefig(out, dpi=150)
print("OK ->", out.name)
