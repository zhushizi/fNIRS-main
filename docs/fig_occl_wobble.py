# -*- coding: utf-8 -*-
"""阻断记录的 TOI 稳定性 —— 必须在 online 的**生产配置**下评估。

产物：docs/验证图/阻断_波动来源.png
数据：software/srs_cal/原始归档/*.srsraw（2026-10-22 ~ 23）

⚠️ 本脚本的第一版（2026-10-23 晚）用 prev 扣暗 + 5 周期窗口做分析，
   而 online 实际跑的是 **interp 扣暗 + N=20 滑动窗**。两者结论不同：
   前者把 0.0868 Hz 的窄带干扰原样留在数据里，「按光强分组」实际上
   分的是干扰的相位。更正后保留对比，作为「分析配置必须与生产一致」的证据。
"""
import sys, json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent / "software"
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tools"))
from srs_cal import archive, hardware as hw
from srs_cal.robust import aggregate
from srs_cal.darksub import DarkSubtractor, MODE_INTERP, MODE_PREV
from fnirs_pipeline.srs import compute_toi

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False
# ⚠️ U+2082（₂）不在 Microsoft YaHei 里，实测渲染成空心方框。一律写 %O2。

ARC = ROOT / "srs_cal" / "原始归档"
CAL = json.loads((ROOT / "srs_cal" / "标定参数" /
                  "标定_v5_探头优化_双波长.json").read_text(encoding="utf-8"))
WL, RHO = hw.WAVELENGTHS_ACTIVE_NM, hw.RHO_CM
KEYS = [(w, r) for w in WL for r in RHO]
# dark_level=None —— 与 online 一致：暗电平已在 DarkSubtractor 里逐周期扣掉
GEOM = hw.geometry(gain_correction=np.asarray(CAL["gain_correction"], float),
                   k=CAL.get("k", 1.0), b=CAL.get("b", 0.0),
                   wavelengths_nm=WL, dark_level=None)


def run(nm, mode=MODE_INTERP, avg=20, channel=None):
    """复刻 online 的链路：DarkSubtractor → 滑动窗 → 稳健聚合 → compute_toi。

    ⚠️ **必须限定一个采集通道**（2026-10-29）。这里只有一个 `DarkSubtractor`，
       而它只认 (λ, ρ)、不认通道 —— 多通道归档不筛就会把另一通道的暗时隙
       扣到这一通道的信号上。`channel=None` 取归档里出现的第一个通道。
       （阻断实验那几份都是单通道采的，所以此前没出事；但脚本不该靠
       "输入恰好是单通道"这个巧合。）
    """
    ds = DarkSubtractor(mode=mode)
    win = {k: [] for k in KEYS}
    out, fresh, t0 = [], set(), None
    frames = archive.replay(ARC / f"{nm}.srsraw")
    if channel is None:
        channel = next((int(f.channel) for f in frames), 1)
    for f in (f for f in frames if int(f.channel) == int(channel)):
        if t0 is None:
            t0 = f.t_host
        if f.is_dark:
            for wl, rho, t, net in ds.feed_dark(f.distance_cm, f.t_host,
                                                float(f.adc)):
                if (wl, rho) in win:
                    win[(wl, rho)].append(net); fresh.add((wl, rho))
            if len(fresh) == len(KEYS):
                if all(len(win[k]) >= avg for k in KEYS):
                    ags = {k: aggregate(win[k][-avg:]) for k in KEYS}
                    I = np.array([[ags[(w, r)].value for r in RHO] for w in WL])
                    if np.all(np.isfinite(I)):
                        r = compute_toi(I, GEOM)
                        out.append((f.t_host - t0, I[0, 1], r.toi,
                                    len(r.flags) > 0))
                fresh = set()
        elif (f.wavelength_nm, f.distance_cm) in win:
            for wl, rho, t, net in ds.feed_signal(f.wavelength_nm,
                                                  f.distance_cm, f.t_host,
                                                  float(f.adc)):
                win[(wl, rho)].append(net); fresh.add((wl, rho))
    a = np.array([o for o in out if np.isfinite(o[2])], dtype=float)
    return a[:, 0], a[:, 1], a[:, 2], a[:, 3].astype(bool)


def slope_ci(T, Y, avg):
    """线性斜率 %O2/min，并按滑动窗重叠修正自由度。

    滑动窗相邻点共享 avg−1 个样本，**不独立**。有效样本数按 n/avg 计，
    这是保守估计；不修正会把 SE 低估约 √avg = 4.5 倍。
    """
    k, _ = np.polyfit(T, Y, 1)
    resid = Y - np.polyval(np.polyfit(T, Y, 1), T)
    n_eff = max(len(T) / avg, 3)
    se = np.std(resid) / (np.std(T) * np.sqrt(n_eff))
    return k * 60, se * 60


NM = "20261023_150943_001"
T, I4, TOI, FL = run(NM)                               # 生产配置
Tp, I4p, TOIp, _ = run(NM, MODE_PREV, 5)               # 第一版分析用的配置

fig = plt.figure(figsize=(13.2, 10.4), dpi=130)
fig.suptitle("阻断记录 20261023_150943_001（手臂勒紧，勒红）的 TOI 稳定性\n"
             "评估必须用 online 的生产配置：interp 扣暗 + N=20 滑动窗",
             fontsize=13, fontweight="bold", y=0.975)

# ---- ① 两种配置下的同一份数据 ----
ax1 = fig.add_subplot(3, 1, 1)
ax1.plot(Tp, TOIp, "-", color="#c0392b", lw=1, alpha=.75,
         label=f"prev 扣暗 + 5 周期（**非生产配置**）  n={len(Tp)}  "
               f"std {TOIp.std():.2f}")
ax1.plot(T, TOI, "-", color="#1b7837", lw=1.8,
         label=f"interp 扣暗 + N=20（online 默认）  n={len(T)}  "
               f"std {TOI.std():.2f}")
ax1.set_ylabel("TOI  (%O2)")
ax1.set_ylim(55, 100)
ax1.set_title("① 同一份原始数据，换扣暗模式与窗口长度，TOI std 相差 "
              f"{TOIp.std()/TOI.std():.1f} 倍。"
              "0.0868 Hz 窄带干扰在短窗 + prev 下原样留着",
              fontsize=11, loc="left")
ax1.legend(fontsize=9, loc="lower left")
ax1.grid(alpha=.3)

# ---- ② 生产配置下的 TOI 与光强 ----
ax2 = fig.add_subplot(3, 1, 2)
k, se = slope_ci(T, TOI, 20)
ax2.plot(T, TOI, "o-", color="#1b7837", ms=3, lw=1.2, label="TOI")
ax2.plot(T, np.polyval(np.polyfit(T, TOI, 1), T), "-", color="k", lw=2,
         label=f"线性趋势 {k:+.2f} ± {se:.2f} %O2/min（已按窗重叠修正自由度）")
ax2.set_ylabel("TOI  (%O2)", color="#1b7837")
ax2.tick_params(axis="y", labelcolor="#1b7837")
ax2b = ax2.twinx()
ax2b.plot(T, I4, "-", color="#777", lw=1, alpha=.7)
ax2b.set_ylabel("670nm/4cm 扣暗后计数", color="#777")
ax2b.tick_params(axis="y", labelcolor="#777")
ax2.set_xlabel("时间 (s)")
ax2.set_title(f"② 生产配置：全程 std {TOI.std():.2f}，趋势 {k:+.2f} ± {se:.2f} %O2/min"
              f"（205 s 合计约 {k*205/60:+.1f} %O2）。"
              "动脉阻断应为 −20~−30 —— 差一个数量级，判为静脉型",
              fontsize=11, loc="left")
ax2.legend(fontsize=9, loc="lower left")
ax2.grid(alpha=.3)

# ---- ③ 跨记录：生产配置下的全程 std ----
ax3 = fig.add_subplot(3, 1, 3)
RECS = [("20261023_111030_001", "111030 A2体模", False),
        ("20261022_141147_001", "141147 体模", False),
        ("20261022_164708_001", "164708 体模", False),
        ("20261023_113333",     "113333 在体", True),
        ("20261023_131237",     "131237 在体", True),
        ("20261023_134037",     "134037 在体", True),
        ("20261023_150943_001", "150943 手臂勒紧", True),
        ("20261023_103952_001", "103952 在体", True),
        ("20261023_134407_001", "134407 阻断", True),
        ("20261023_150038_001", "150038 在体", True)]
for nm, lab, invivo in RECS:
    try:
        t, i4, toi, fl = run(nm)
    except Exception:
        continue
    if toi.size < 20:
        continue
    x, y = float(np.median(i4)), float(toi.std())
    ax3.plot(x, y, "o" if invivo else "s", ms=11,
             color="#c0392b" if invivo else "#2166ac", mec="k", mew=.6, zorder=3)
    ax3.annotate(f"{lab}\nflag {100*fl.mean():.0f}%", (x, y),
                 textcoords="offset points", xytext=(10, -6), fontsize=8)
ax3.plot([], [], "s", color="#2166ac", mec="k", ms=10, label="静止体模")
ax3.plot([], [], "o", color="#c0392b", mec="k", ms=10, label="在体（手臂）")
ax3.set_xscale("log"); ax3.set_yscale("log")
ax3.set_xlim(8e2, 1e6)
ax3.set_xlabel("670nm / 4cm 扣暗后计数（全程中位）")
ax3.set_ylabel("TOI std  (%O2)")
ax3.set_title("③ 光强不是决定因素：~2,300 counts 的静止体模做到 1.86/2.39，"
              "同等光强的在体是 10.52；而手臂在 16,657 counts 下是 1.20。"
              "括号内为该记录被质量层标记的点占比",
              fontsize=11, loc="left")
ax3.legend(fontsize=9, loc="lower left")
ax3.grid(alpha=.3, which="both")

fig.text(0.012, 0.012,
         "结论：① 勒红是静脉淤血（动脉仍在供氧），生理上 TOI 本就不该大幅下降；"
         "② 生产配置下这份记录的 TOI std 1.20，已在噪声底量级，"
         "屏幕上的「抖」是相对体模 0.07 而言；③ 评估任何改动都必须用生产配置复算。",
         fontsize=10, color="#333")

fig.tight_layout(rect=[0, 0.035, 1, 0.945])
out = HERE / "验证图" / "阻断_波动来源.png"
out.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(out, facecolor="white")
print(f"已保存 {out}")
print(f"  生产配置 interp+N=20: n={len(T)} std {TOI.std():.2f} "
      f"趋势 {k:+.2f} ± {se:.2f} %O2/min")
print(f"  第一版   prev+5     : n={len(Tp)} std {TOIp.std():.2f}")
