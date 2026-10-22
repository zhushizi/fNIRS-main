# -*- coding: utf-8 -*-
"""双解码器 —— 实况架构图（as-built，不是设计稿）。

框里的行数查自 wc -l，实测值来自真实归档重放。
与 docs/双解码器架构.md §二 的 ASCII 设计图不同之处用 ★ 标出。
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

ROOT = Path(r"D:\WORK_space\Code_WS\code_localized\fNIRS-main")

SURF, INK, INK2, INK3, RULE = "#fcfcfb", "#0b0b0b", "#52514e", "#8b8a84", "#e3e2dd"
OK, BAD, NEU, WARN = "#1baf7a", "#eb6834", "#2a78d6", "#c9870f"
GREY = "#f4f3f0"
plt.rcParams.update({
    "font.sans-serif": ["Microsoft YaHei", "SimHei", "DengXian", "sans-serif"],
    "axes.unicode_minus": False, "figure.facecolor": SURF, "text.color": INK})

# 画布：轴填满整张图，纵横单位密度一致（6.67 单位/英寸）。
# ⚠️ 上一版用 subplots 默认轴域，内容溢出轴外被裁掉（L4 整框丢失）—— 故改成 add_axes。
W, H = 100.0, 184.0
fig = plt.figure(figsize=(15.0, 27.6))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W); ax.set_ylim(0, H); ax.axis("off")

LH = 2.00          # 正文行高
HEAD = 3.4         # 标题占高
PADT, PADB = 1.4, 1.6


def box(x, y_top, w, title, lines, edge=INK3, fill="#ffffff",
        tsize=11, lsize=9.0, badge=None, bcol=OK):
    """y_top 为框顶，返回框底 y。"""
    h = PADT + HEAD + LH * len(lines) + PADB
    ax.add_patch(FancyBboxPatch((x, y_top - h), w, h,
                                boxstyle="round,pad=0.3", linewidth=1.5,
                                edgecolor=edge, facecolor=fill,
                                clip_on=False))
    ax.text(x + 1.3, y_top - PADT - 1.0, title, fontsize=tsize,
            fontweight="bold", color=INK, va="top")
    for i, ln in enumerate(lines):
        col = BAD if ln.startswith("▲") else (
            WARN if ln.startswith("！") else INK2)
        wt = "bold" if (ln.startswith("▲") or ln.startswith("！")) else "normal"
        ax.text(x + 1.3, y_top - PADT - HEAD - LH * i - 0.3, ln,
                fontsize=lsize, color=col, va="top", fontweight=wt)
    if badge:
        ax.text(x + w - 1.3, y_top - PADT - 1.4, badge, fontsize=8.6,
                color="white", fontweight="bold", ha="right", va="top",
                bbox=dict(boxstyle="round,pad=0.32", facecolor=bcol,
                          edgecolor="none"))
    return y_top - h


def arrow(x0, y0, x1, y1, col=INK3, lw=1.6, ls="-"):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle="-|>",
                                 mutation_scale=15, linewidth=lw, color=col,
                                 linestyle=ls, shrinkA=0, shrinkB=0))


GAP = 4.6
ax.text(2.5, H - 2.0, "双解码器实况架构（as-built）　2026-10-22",
        fontsize=17, fontweight="bold", va="top")
ax.text(97.5, H - 2.4, "★ = 与 §二 设计稿不同之处（实测后改的）　"
        "▲ = 缺口　！= 使用限制",
        fontsize=10, color=INK3, ha="right", va="top")
y = H - 7.5

# ---------------------------------------------------------------- L0
y = box(4, y, 92, "L0　设备（未改，也改不了）", [
    "1 源 + 2 探　ρ = 3.0 / 4.0 cm　×　5 波长 670 / 770 / 810 / 850 / 950",
    "上电自发数据，不响应启停，只接受档位指令 0x00~0xFF"],
    edge=INK3, fill=GREY, badge="厂家", bcol=INK3)
arrow(50, y, 50, y - GAP)
ax.text(51.4, y - GAP / 2, "串口字节流 115200", fontsize=8.6, color=INK3,
        va="center")
y -= GAP

# ---------------------------------------------------------------- L1
y = box(4, y, 92, "L1　协议层　srs_cal/protocol.py（351 行）", [
    "字节流同步 55AA · 校验 · 字段解码 · 假帧防护 · 链路计数",
    "★ 新增 t_host（perf_counter，到达即打戳）+ cycle_id　→ 解除 R2",
    "   本轮唯一改动的已有代码，也是唯一知道时间的层",
    "实测：周期 1521.53 ms（0.657 周期/秒），抖动 MAD 0.04 ms —— 时基很稳"],
    edge=OK, badge="√ 第1条")
arrow(50, y, 50, y - GAP)
y -= GAP

# ---------------------------------------------------------------- L2
y = box(4, y, 92, "L2　原始归档　srs_cal/archive.py（262 行，新）", [
    "两条解码链唯一的共同输入 · 不可变 · 五波长全存（含停用的 810/850/950）",
    "34 字节定长记录 + JSON 头 · 按时长轮转 1 h · 总量上限 5 GB",
    "写失败只计数不中断 —— 测量不能被归档拖垮",
    "▲ 缺口：只有 online.py 接入，acquire.py（标定采集）尚未归档"],
    edge=OK, badge="√ 第2条")
arrow(50, y, 50, y - GAP)
y -= GAP

# ---------------------------------------------------------------- darksub
y = box(4, y, 92, "插值扣暗　srs_cal/darksub.py（129 行，新）"
                  "　——　两条链共用的唯一预处理", [
    "按 t_host 在前后两个暗帧之间插值，而不是逐周期扣同一个暗值",
    "★ 为什么：噪声主成分是 0.0868 Hz 窄带干扰，暗帧与信号帧最多差 1.5 s = 47° 相位，",
    "   逐周期扣暗消不掉。代价是 1 个周期的延迟"],
    edge=OK, badge="√ 第7条")
arrow(50, y, 50, y - GAP)
y -= GAP

# ---------------------------------------------------------------- L2.5
y = box(4, y, 92, "L2.5　质量层　srs_cal/quality.py（313 行，新 · 只读不改数据）", [
    "A 数据判据：饱和 / 非正 / 尖峰 5×MAD / 暗电平 / 链路计数",
    "B 物理判据：tHb ∈ 20~120 µM · TOI ∈ 0~100% · 浓度非负　—— 补 R1 的零自检",
    "C 停用波长监视：810/850/950 盯加性项 C 是否回来（区分反演通道与监视通道）",
    "实测判别力：稳定段 0% 误报，已知坏段 14/14 检出"],
    edge=OK, badge="√ 第6条")
y_split = y
ax.text(50, y - 2.4, "flags —— 两条链都读，都不写", fontsize=9, color=INK3,
        ha="center")
arrow(50, y, 27, y - GAP - 1.4)
arrow(50, y, 73, y - GAP - 1.4)
y -= GAP + 1.4

# ---------------------------------------------------------------- L3A / L3B
yA = box(2, y, 47, "L3A　SRS 解码器 —— 绝对量", [
    "fnirs_pipeline/srs.py（541）+ srs_cal/online.py（798）",
    "+ srs_cal/robust.py（93）",
    "",
    "预处理（自己的）：",
    "  1 插值扣暗",
    "  2 稳健聚合：偏离中位 >5×MAD 剔除后取均值",
    "     ★ 不是中位数 —— 实测选型，中位数效率更差",
    "  3 合并 N = 20 周期（30 s）　★ 原计划 10~15",
    "  × 禁止高通 —— 会毁掉绝对值",
    "",
    "配置：srs_cal/hardware.py　G(λ)、k、b",
    "标定链：run_srs_v1.py → 标定_vN.json",
    "        波长集带校验（解除 R8）",
    "",
    "输出：TOI 绝对值 · [HbO]/[HbR]/tHb · 光谱残差",
    "      · μa / μeff / slope · 未标定标志（不可被下游丢弃）",
    "",
    "▲ k/b 仍未对标 → 绝对值尚未验证（R5，第 11 条）"],
    edge=OK, badge="√ 第3·4·7条", lsize=8.8)

yB = box(51, y, 47, "L3B　MBLL 解码器 —— 相对量", [
    "mbll_decoder/（本轮新建）　hardware·decoder·run·live",
    "live.py：攒缓冲+整段重算（零相位滤波没法流式化）",
    "",
    "预处理（自己的）：",
    "  1 插值扣暗",
    "  2 按 t_host 重采样到均匀栅格",
    "     （丢帧会留空洞，不能当等间隔）",
    "  3 ΔOD = −log10(I / I_ref)，参考取全段均值",
    "  4 带通 0.01 ~ 0.05 Hz　★ 原定 0.1 Hz",
    "     干扰正在 0.0868 Hz 上",
    "配置：自带 hardware.py —— 刻意不 import config",
    "      670/770 · ρ=3.0 · DPF Scholkmann",
    "      × 无短距→无 SSR　× 2 波长→跳过 Cyt",
    "标定链：无（相对量不需要）",
    "输出：ΔHbO / ΔHbR · 不再有 rSO2 · 不再需要基线窗",
    "      实测噪声底 0.124 / 0.075 µM（体模静置 644 s）",
    "",
    "！开头 300 s 这一格是空的：高通 0.01 Hz 起步所需"],
    edge=OK, badge="√ 第8条", lsize=8.8)

y = min(yA, yB)
arrow(25, yA, 25, y - GAP)
arrow(75, yB, 75, y - GAP)
y -= GAP

# ---------------------------------------------------------------- L4
y = box(6, y, 88, "L4　展示 / 导出（srs_cal/online.py 的 GUI）　—— 三格并排", [
    "① TOI（SRS 绝对量，最近 60 s）　② 各波长 μa　③ ΔHbO/ΔHbR（MBLL，整段记录）",
    "③ 刻意不与 ①② 共用 x 轴：上面看实时窗，下面看整段，强行共用必有一边看不清",
    "√ 质量标志 + 未标定标志　√ 归档开关（界面可关）",
    "",
    "核心不变量：界面上的 ΔHb 与 `python -m mbll_decoder.run` 逐点相同（实测差 0）",
    "MBLL 三层 try 保护：import / 分流 / 刷新。坏掉只摘这一格并标红，TOI 照常出点",
    "",
    "★ 硬性禁止：两条链不得共用任何预处理中间量。rSO2 取消后，",
    "   「用 TOI 设 MBLL 基线」已结构上不可能，不再靠纪律保证。"],
    edge=OK, badge="√ 已补齐")

# ---------------------------------------------------------------- 脚注
ax.text(4, y - 3.2, "隔离验证", fontsize=10, fontweight="bold", color=INK)
ax.text(14, y - 3.2,
        "mbll_decoder 的真实 import（AST 解析，不是搜文本）只有 "
        "fnirs_pipeline.mbll_core / numpy / scipy —— 没有 config、"
        "没有 online_android、没有 fnirs_pipeline.mbll",
        fontsize=9, color=INK2)
ax.text(4, y - 6.0, "自检 139/139 通过（t1~t16）　·　行数取自 wc -l　·　"
        "实测值全部来自真实归档重放，不是估计",
        fontsize=8.8, color=INK3)

out = ROOT / "docs" / "双解码器架构图.png"
fig.savefig(out, dpi=140)
print("内容底 y =", round(y - 6.0, 1), "（须 > 0）")
print(f"OK -> {out.name}")
