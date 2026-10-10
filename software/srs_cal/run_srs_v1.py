# -*- coding: utf-8 -*-
"""v1 标定：跑通整条 SRS 并给出验证方法。

反演波长跟随 `hardware.WAVELENGTHS_ACTIVE_NM`（当前 670/770/810/850）。

    python -m srs_cal.run_srs_v1                    # 用已采的真实数据
    python -m srs_cal.run_srs_v1 --fig              # 另出验证图到 docs/验证图/

这一版的定位：**在硬件修好 810 之前，先证明其余环节是通的。**
它不产出可出厂的 G(λ) —— 只有 4 块、每块 1 次，样本远不够（见文末
「12 块到齐后怎么验」）。它要回答的是另一个问题：除了 810，这条链路
（协议 → 光强 → 斜率 → μeff → μa）还有没有别的系统性毛病。

为什么用 μa 回收率当验证指标，而不是 TOI
----------------------------------------
体模是干材料，不含血红蛋白，TOI 在体模上没有意义。但 μa 有 ——
厂家对每块都给了认证 μa，所以可以直接问：**反演出来的 μa 等于认证值吗？**
这比看 G 的离散更硬：G 是拟合出来的自由参数，μa 是有独立真值的被测量。
"""

from __future__ import annotations

import argparse
import glob
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))

from srs_cal import hardware as hw        # noqa: E402
from srs_cal import phantom               # noqa: E402
from srs_cal.calibrate import calibrate_gain   # noqa: E402
from srs_cal.forward import predicted_slope    # noqa: E402

WL = hw.WAVELENGTHS_ACTIVE_NM                  # 670 / 770 / 850
RHO = hw.RHO_CM
N = phantom.REFRACTIVE_INDEX
DATA_DIR = _HERE / "标定采集"
DOCS_FIG = _HERE.parent.parent / "docs" / "验证图"


# ---------------------------------------------------------------------------
def load_measured(data_dir: Path = DATA_DIR,
                  quiet: bool = False) -> tuple[dict, dict, dict]:
    """把「逐帧」表读成 {块: {(λ,ρ): [逐帧值]}}，暗采单独返回。

    读逐帧而不是汇总，是为了能算批均值标准误 —— 相邻帧相关时
    std/√n 会把不确定度低估几十倍（2026-10-09 实测 r=0.07~0.30）。

    按**列名**取值，不按列号 —— 「逐帧」表的列变过一次（新增 adc_net），
    按列号读会在换固件后静默读错一列。两种格式都要能读：

        A 旧      adc，**无** λ=0 行   ← 原始值；暗来自单独的 DARK 块文件
        B 过渡    adc，**有** λ=0 行   ← 信号已逐周期扣暗（实测与「明细」逐行相等）
        C 新      adc_raw + adc_net    ← 取 adc_net

    判据按这个顺序：有 adc_net 就是 C；否则看有没有 λ=0 行，有就是 B，
    没有就是 A。只按列名判会把 B 当成 A 再扣一次暗。

    返回 (blocks, dark, era)。blocks **一律是扣暗后的值**，下游不得再扣。
    era 记每个块来自哪个时代：'slot' = 有暗时隙，'legacy' = 旧固件。
    ⚠️ 两个时代**不能混算**：旧固件只能整场扣一次暗，810 的 4cm 因此
       偏 2~7 倍；加暗时隙后同一块的 810 偏离回到 ±0.03。混在一起
       G(810) 的块间离散会被拖到 0.34，整条标定作废。
    """
    import openpyxl

    blocks: dict[str, dict] = {}          # 只放当前选中的时代
    by_era: dict[str, dict] = {"slot": {}, "legacy": {}}
    dark: dict = {}
    pend: list[tuple[str, dict]] = []     # 旧格式，待全局 DARK 到齐
    era: dict[str, set] = {}              # 块 -> {'slot','legacy'}
    seen: set = set()                     # (块, take, 时间戳) 全局去重
    for fp in sorted(data_dir.glob("标定采集_*.xlsx")):
        wb = openpyxl.load_workbook(fp, read_only=True, data_only=True)
        if "逐帧" not in wb.sheetnames:
            wb.close()
            continue
        rows = list(wb["逐帧"].values)
        # 「明细」给出每个 take 的时间戳 —— 用它做全局去重。
        # ⚠️ 采集软件是**累积保存**：每存一次都把本次会话已有的 take 全写进去，
        #    所以后一个文件常常是前一个的超集。不去重会把同一次采集算好几遍，
        #    均值不变，但批均值标准误会被虚假地缩小 √n 倍。
        stamp = {}
        if "明细" in wb.sheetnames:
            for r in list(wb["明细"].values)[1:]:
                if r and r[0] is not None:
                    stamp[(r[0], r[1])] = r[2]
        wb.close()
        if not rows:
            continue
        hdr = {str(c): i for i, c in enumerate(rows[0]) if c}
        need = ("block", "wavelength_nm", "distance_cm")
        if not all(k in hdr for k in need):
            print(f"  ⚠️ 跳过 {fp.name}：「逐帧」表头不认识 {rows[0]}")
            continue
        # 新表优先用 adc_net（已扣暗）；旧表只有 adc
        col = hdr.get("adc_net") or hdr.get("adc")
        if col is None:
            print(f"  ⚠️ 跳过 {fp.name}：既无 adc_net 也无 adc 列")
            continue
        ib, iw, ir = hdr["block"], hdr["wavelength_nm"], hdr["distance_cm"]
        this: dict[str, dict] = {}
        this_dark: dict = {}
        has_slot = False
        for r in rows[1:]:
            if not r or r[ib] is None or r[col] is None:
                continue
            w = float(r[iw])
            if w == 0.0 or r[ib] == "DARK":
                has_slot = has_slot or w == 0.0
                src = hdr.get("adc_raw", col)
                if r[src] is None:
                    continue
                this_dark.setdefault(float(r[ir]), []).append(float(r[src]))
                continue
            if (r[ib], r[1], stamp.get((r[ib], r[1]))) in seen:
                continue                      # 这一次采集在前面的文件里已经收过
            this.setdefault(r[ib], {}).setdefault(
                (w, float(r[ir])), []).append(float(r[col]))

        new_takes = {(b, t, ts) for (b, t), ts in stamp.items()
                     if b in this}
        dup = len(stamp) - len(new_takes)
        seen |= new_takes
        fmt = "C 新" if "adc_net" in hdr else ("B 过渡" if has_slot else "A 旧")
        if fmt == "A 旧":
            # 只有这一种需要在这里扣：暗在**另一个文件**的 DARK 块里，
            # 所以用全局累积的 dark，而不是本文件的。
            pend.append((fp.name, this))
        else:
            for b, combos in this.items():
                tgt = by_era["slot"].setdefault(b, {})
                for k, vals in combos.items():
                    tgt.setdefault(k, []).extend(vals)
                era.setdefault(b, set()).add("slot")
        for rho, vals in this_dark.items():
            dark.setdefault(rho, []).extend(vals)
        if not quiet:
            print(f"  {fp.name}  [{fmt}]  新增 {sorted(this)}"
                  + (f"  （跳过 {dup} 个重复 take）" if dup else ""))

    # 旧格式：暗来自单独的 DARK 文件，必须等全部读完才知道
    if pend:
        d = {rho: float(np.mean(v)) for rho, v in dark.items()}
        for name, this in pend:
            if not d:
                print(f"  ⚠️ 跳过 {name}：旧格式且全目录都没有 DARK 块")
                continue
            for b, combos in this.items():
                tgt = by_era["legacy"].setdefault(b, {})
                for (w, rho), vals in combos.items():
                    off = d.get(rho, 0.0)
                    tgt.setdefault((w, rho), []).extend(v - off for v in vals)
                era.setdefault(b, set()).add("legacy")
    # ⚠️ 同一块可能两个时代都有（旧固件采过、换固件后又采了）。
    #    **绝不能合并** —— 旧的 810 是坏的，合进来会把它拖回去。
    #    这里只返回原始的分时代结构，由调用方挑。
    return by_era, dark, era


def batch_se(x, n_batch: int = 10) -> float:
    """批均值法的均值标准误。相邻帧相关时 std/√n 不成立。"""
    x = np.asarray(x, float)
    k = len(x) // n_batch
    if k < 2:
        return float("nan")
    m = x[:k * n_batch].reshape(n_batch, k).mean(axis=1)
    return float(m.std(ddof=1) / np.sqrt(n_batch))


def measured_slopes(blocks: dict, dark: dict) -> tuple[list, np.ndarray, np.ndarray]:
    """→ (块码, 斜率(n_blk,n_wl), 斜率不确定度(n_blk,n_wl))。

    ⚠️ `blocks` 进来时**已经扣过暗**（由 load_measured 按文件格式完成）。
       这里不能再扣 —— 对新格式会扣两次。`dark` 只用于留档/报告。
    """
    codes = sorted(blocks, key=lambda c: phantom.BLOCK_CODES.index(c))
    S = np.full((len(codes), WL.size), np.nan)
    U = np.full_like(S, np.nan)
    for i, c in enumerate(codes):
        for j, w in enumerate(WL):
            a3 = np.asarray(blocks[c][(w, RHO[0])], float)
            a4 = np.asarray(blocks[c][(w, RHO[1])], float)
            S[i, j] = np.log10(a3.mean() / a4.mean()) / (RHO[1] - RHO[0])
            U[i, j] = np.hypot(batch_se(a3) / (a3.mean() * np.log(10)),
                               batch_se(a4) / (a4.mean() * np.log(10)))
    return codes, S, U


# ---------------------------------------------------------------------------
def recover_mua(slope: np.ndarray, musp: np.ndarray, gain=None) -> np.ndarray:
    """斜率 → μeff → μa。用产品代码本身，不是复制品。

    μs′ 取体模认证值 —— 体模上 μs′ 是已知的，用它可以把「斜率测得准不准」
    与「μs′ 假设对不对」分开。在体时 μs′ 要靠先验，那是另一个误差源。
    """
    from fnirs_pipeline.srs import mua_from_mueff, slope_to_mueff

    s = np.asarray(slope, float)
    if gain is not None:
        s = s - np.asarray(gain, float)
    return mua_from_mueff(slope_to_mueff(s, RHO), musp)


def _fmt(v, spec="+.4f"):
    return "  n/a " if not np.isfinite(v) else format(v, spec)


# ---------------------------------------------------------------------------
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="v1 SRS 标定与验证")
    ap.add_argument("--data", default=str(DATA_DIR))
    ap.add_argument("--fig", action="store_true", help="另出验证图到 docs/验证图/")
    ap.add_argument("--legacy", action="store_true",
                    help="把旧固件（无暗时隙）的数据也算进来。仅供对照："
                         "两个时代的暗电平处理不同，810 尤其不可比。")
    ap.add_argument("--emit-cal", metavar="PATH", nargs="?",
                    const=str(_HERE / "标定_v1.json"),
                    help="把 G(λ)+暗电平写成标定文件，供 online.py 加载")
    a = ap.parse_args(argv)

    # Windows 控制台默认 GBK，打不出 ²、λ 这类字符。报告里这些符号是内容
    # 的一部分（μa、R²、%O₂），不能为了迁就终端改掉。
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:                                            # noqa: BLE001
        pass

    by_era, dark, era = load_measured(Path(a.data))
    # 两个时代不能混算，**同一块也不行** —— 旧固件只能整场扣一次暗，
    # 810 偏 2~7 倍；与新数据合并会把它拖回去。
    if by_era["slot"] and not a.legacy:
        blocks = by_era["slot"]
        both = sorted(b for b, e in era.items() if e == {"slot", "legacy"})
        only_old = sorted(b for b, e in era.items() if e == {"legacy"})
        if both:
            print(f"\n  ℹ️ {', '.join(both)} 新旧都有 —— 只用新（有暗时隙）的那部分。")
        if only_old:
            print(f"  ℹ️ 排除只有旧数据的块：{', '.join(only_old)}")
        print("     要强行合并用 --legacy（仅供对照，不可用于标定）。")
    elif a.legacy:
        blocks = {}
        for e in ("legacy", "slot"):
            for b, v in by_era[e].items():
                tgt = blocks.setdefault(b, {})
                for k, vals in v.items():
                    tgt.setdefault(k, []).extend(vals)
        print("\n  ⚠️ --legacy：新旧混算，810 不可信，结果仅供对照。")
    else:
        blocks = by_era["legacy"]
        print("\n  ⚠️ 全部是旧固件数据（无暗时隙）。810 的结果不可信 ——"
              "当时的「泄漏」已证实是暗电平未扣准，见 hardware.py。")
    if not blocks:
        print(f"没找到采集数据：{a.data}")
        return 1
    codes, S, U = measured_slopes(blocks, dark)
    mua_c, musp_c = phantom.interpolate_to(WL)
    sel = [phantom.BLOCK_CODES.index(c) for c in codes]
    nb = len(codes)

    print("=" * 74)
    print(f"v1 标定 —— 波长 {'/'.join(f'{w:.0f}' for w in WL)} nm"
          f"，距离 {RHO[0]:.0f}/{RHO[1]:.0f} cm，n={phantom.REFRACTIVE_INDEX}")
    print("=" * 74)
    print(f"  实测块：{', '.join(codes)}（共 {nb} 块，每块 1 次）")

    # ---- ① G(λ) -----------------------------------------------------------
    per = np.array([S[i] - predicted_slope(mua_c[sel[i]], musp_c[sel[i]], RHO, N)
                    for i in range(nb)])
    G = per.mean(axis=0)
    disp = per.std(axis=0, ddof=1)
    print("\n① G(λ)")
    print("   λ      G(λ)      块间离散   平均不确定度   离散/不确定度")
    for j, w in enumerate(WL):
        u = np.nanmean(U[:, j])
        print(f"  {w:.0f}  {G[j]:+9.4f}  {disp[j]:10.4f}  {u:12.4f}  {disp[j]/u:12.1f} ×")
    print("\n   各块单独的 G：")
    for i, c in enumerate(codes):
        print(f"     {c}  " + "  ".join(f"{w:.0f}:{per[i, j]:+.4f}"
                                        for j, w in enumerate(WL)))

    # ---- ② 留一验证 -------------------------------------------------------
    print("\n② 留一验证（用 3 块定 G，预测第 4 块）")
    print("   留出块    λ     预测斜率   实测斜率     误差     折合 μa 误差")
    errs, mua_errs = [], []
    for i, c in enumerate(codes):
        m = np.arange(nb) != i
        G_loo = per[m].mean(axis=0)
        s_pred = predicted_slope(mua_c[sel[i]], musp_c[sel[i]], RHO, N) + G_loo
        mua_hat = recover_mua(S[i], musp_c[sel[i]], G_loo)
        for j, w in enumerate(WL):
            e = S[i, j] - s_pred[j]
            em = (mua_hat[j] - mua_c[sel[i], j]) / mua_c[sel[i], j] * 100
            errs.append(e)
            mua_errs.append(em)
            print(f"     {c if j == 0 else '  ':<6} {w:.0f}  {s_pred[j]:10.4f}"
                  f" {S[i, j]:10.4f} {e:+10.4f} {em:+10.1f} %")
    errs = np.array(errs)
    mua_errs = np.array(mua_errs)
    print(f"\n   斜率预测误差 RMS {np.sqrt((errs**2).mean()):.4f}"
          f"（最大 {np.abs(errs).max():.4f}）")
    print(f"   μa 回收误差   RMS {np.sqrt((mua_errs**2).mean()):.1f} %"
          f"（最大 {np.abs(mua_errs).max():.1f} %）")

    # ---- ③ μa 回收趋势 ----------------------------------------------------
    print("\n③ μa 回收趋势（这是「用其他体模验证趋势」要看的那张图）")
    print("   用全部块定 G，再反演各块 μa，与认证值比")
    print("    块    λ    认证 μa    反演 μa    相对误差")
    xs, ys = [], []
    for i, c in enumerate(codes):
        mua_hat = recover_mua(S[i], musp_c[sel[i]], G)
        for j, w in enumerate(WL):
            t, h = mua_c[sel[i], j], mua_hat[j]
            xs.append(t)
            ys.append(h)
            print(f"    {c if j == 0 else '  ':<4} {w:.0f}  {t:9.5f}  {h:9.5f}"
                  f"  {(h - t) / t * 100:+8.1f} %")
    xs, ys = np.array(xs), np.array(ys)
    k, b = np.polyfit(xs, ys, 1)
    r2 = 1 - ((ys - (k * xs + b)) ** 2).sum() / ((ys - ys.mean()) ** 2).sum()
    print(f"\n   回归 反演μa = {k:.4f} × 认证μa {b:+.5f}   R² = {r2:.4f}")
    print(f"   理想值：斜率 1.0000、截距 0、R² 1.0000")
    print(f"   → 斜率偏离 {abs(k-1)*100:.1f}%，截距 {b:+.5f} cm⁻¹")

    # ---- ③b G 离散 → μa 误差的换算 ----------------------------------------
    print("\n③b 那些 μa 误差是哪来的？—— 两个来源，分开看")
    s_typ = float(np.nanmean(S))
    mueff_typ = np.log(10) * s_typ - 2.0 / float(np.mean(RHO))
    # μa ∝ μeff²，μeff = ln10·(slope−G) − 2/ρ̄  ⇒  dμa/μa = −2·ln10·dG/μeff
    amp = 2 * np.log(10) / mueff_typ
    print(f"   典型斜率 {s_typ:.3f} → μeff {mueff_typ:.3f} cm⁻¹")
    print(f"   灵敏度：ΔG 每 0.01  →  μa 误差 {amp*0.01*100:.1f} %"
          f"（μa ∝ μeff²，放大 2 倍）")

    # ⚠️ 早先这里写「μa 回收误差 100% 来自 G 离散」—— **那是错的**。
    #    即使每块都用自己的 G（离散为 0），误差也不是 0：正演用精确的
    #    Farrell-Patterson 解，反演用渐近式 μeff = ln10·slope − 2/ρ̄，
    #    两者在 ρ=3~4cm 差约 2%，折到 μa（∝μeff²）上放大一倍。
    #    当时 G 离散 0.058 把这一项盖住了，才得出那个错误结论。
    self_err = []
    for i2 in range(nb):
        hat = recover_mua(S[i2], musp_c[sel[i2]], per[i2])   # 用该块自己的 G
        self_err += list((hat - mua_c[sel[i2]]) / mua_c[sel[i2]])
    e_model = float(np.sqrt(np.mean(np.square(self_err))) * 100)
    e_total = float(np.sqrt(np.mean(np.square(
        (np.array(ys) - np.array(xs)) / np.array(xs)))) * 100)
    e_disp = amp * disp.mean() * 100
    print(f"\n   (1) G 的块间离散 {disp.mean():.4f} 贡献      {e_disp:5.1f} %")
    print(f"   (2) 模型失配（正演精确解 vs 反演渐近式） {e_model:5.1f} %"
          "   <- 与 G 无关")
    print(f"   实际 μa 误差 RMS                      {e_total:5.1f} %")
    if e_disp > e_model:
        print("   -> 由 **G 离散** 主导：压 G 的离散就能改善。")
    else:
        print("   -> 由 **模型失配** 主导：再压 G 也降不动 μa 回收率。")
        print("      要动的是反演里的 2/ρ̄ 渐近近似（或改成精确解数值反演）。")
        print("      ⚠️ 但它对 TOI 的影响远小于对 μa —— 它近似是各波长等比例的，")
        print("         在 μa 的谱比值里大部分抵消。别为了 μa 好看去改反演。")

    need = 0.05 / amp
    if disp.mean() <= need:
        print(f"\n   G 离散 {disp.mean():.4f} 已低于「G 对 μa 贡献 <5%」"
              f"所需的 {need:.4f}  OK")
    else:
        print(f"\n   要让 G 对 μa 的贡献 <5%，G 离散须 < {need:.4f}"
              f" —— 比现在好 {disp.mean()/need:.1f} 倍")

    # ---- ③c G 离散的结构 → 真正重要的是哪一半 -----------------------------
    print("\n③c 把 G 的块间差异拆开：平坦偏移 vs 波长相关")
    gm = per.mean()
    row = per.mean(axis=1) - gm          # 每块的整体偏移（与波长无关）
    res = per - (gm + row[:, None] + (per.mean(axis=0) - gm)[None, :])
    s_flat, s_spec = row.std(ddof=1), res.std(ddof=1)
    print("     块     整体偏移        波长相关残差")
    for i, c in enumerate(codes):
        print(f"     {c}   {row[i]:+8.4f}      "
              + " ".join(f"{x:+7.4f}" for x in res[i]))
    print(f"\n   平坦偏移 σ = {s_flat:.4f}   波长相关 σ = {s_spec:.4f}"
          f"   （{s_flat/(s_flat+s_spec)*100:.0f}% 是平坦的）")
    print("   平坦 = 两个探测器通光量之比在变 = 贴合/耦合；")
    print("   波长相关 = 器件的谱响应在变 —— 这一项才是真毛病。")

    tcost = toi_cost(s_flat, s_spec)
    print("\n   两者对最终 TOI 的代价（SO₂=70%，蒙特卡洛）")
    for name, sd in tcost:
        print(f"     {name:<28} TOI 标准差 {sd:5.2f} %O₂")
    print("\n   ⚠️ 注意反了：**小的那一项代价更大**。")
    print("      平坦偏移在 μa 上有 11~13% 误差，但 TOI 只差 1.0 %O₂ ——")
    print("      它对所有波长等量，SO₂ 取的是谱比值，大部分抵消掉了。")
    lin = flat_is_linear()
    print(f"      而且它对 SO₂ 的偏差几乎严格线性（残差 {lin[2]:.3f} %O₂），")
    print(f"      可被人体降氧标定的 k·TOI+b 完全吸收"
          f"（k={lin[0]:.4f}, b={lin[1]:+.3f}）—— 前提是它在标定与测量之间不变。")
    print("   → 优化次序：先压波长相关那一项，不是先压大的那一项。")

    # ---- ④ 丢掉 810 的代价 -------------------------------------------------
    print("\n④ 丢掉 810 的代价（蒙特卡洛，同一噪声水平下比 SO₂ 的离散）")
    cost = so2_cost()
    print("   波长组                    SO₂ 标准差   相对四波长")
    for name, sd, rel in cost:
        print(f"   {name:<24} {sd:8.2f} %O₂  {rel:8.2f} ×")

    # ---- ⑤ 怎么验 ---------------------------------------------------------
    print("\n" + "=" * 74)
    print("⑤ 12 块到齐后怎么验")
    print("=" * 74)
    print(VALIDATION_PROTOCOL)

    if a.fig:
        make_figure(codes, xs, ys, k, b, r2, per, disp, (s_flat, s_spec))
    if a.emit_cal:
        emit_cal(Path(a.emit_cal), G, dark, codes, disp, s_flat, s_spec, tcost)
    return 0


def emit_cal(path: Path, G, dark: dict, codes, disp, s_flat, s_spec,
             tcost) -> None:
    """写标定文件供 online.py 加载。

    刻意把**局限性一起写进文件**（`caveats`），而不是只写数值：
    在线程序启动时会把它原样显示出来。标定文件被当成「已标定」四处流传、
    却没人记得它只有 4 块数据，是这类项目最常见的事故。
    """
    import json
    from datetime import datetime

    # 暗电平只作留档。在线程序用设备每周期的暗时隙，**不读这一项** ——
    # 静态暗几秒就过期（实测漂移时间常数只有几个周期）。
    dl = [[float(np.mean(dark[rho])) if dark.get(rho) else 0.0
           for rho in RHO] for _ in WL]
    doc = {
        "version": "v1-preliminary",
        "created": datetime.now().isoformat(timespec="seconds"),
        "wavelengths_nm": [float(w) for w in WL],
        "rho_cm": [float(r) for r in RHO],
        "refractive_index_phantom": float(N),
        "gain_correction": [float(x) for x in G],
        "dark_level_archive_only": dl,
        "k": 1.0,
        "b": 0.0,
        "source_blocks": list(codes),
        "n_takes_per_block": 1,
        "dispersion_per_wl": [float(x) for x in disp],
        "sigma_flat": float(s_flat),
        "sigma_spectral": float(s_spec),
        # tcost[-1] 是「平坦+波长相关」合计 —— 这才是重复性；
        # 不要误用 so2_cost()（那是比较波长数的，量纲相同但含义不同）
        "expected_toi_sd_pct": float(tcost[-1][1]),
        # ⚠️ caveat 必须由**当次结果**算出来，不能写死。
        #    早先这里固定写着「810 停用」和「由 4 块估计」，
        #    810 恢复、块数变了之后文件里就是假话 —— 而在线程序会把它
        #    原样显示给使用者。
        "caveats": [
            f"仅 {len(codes)} 块（{', '.join(codes)}）、每块 1 次（**无重贴合**）"
            " —— 不是出厂标定",
            f"反演波长 {'/'.join(f'{w:.0f}' for w in WL)} nm",
            f"G 块间离散 {disp.mean():.4f}（各波长 "
            + "、".join(f"{w:.0f}:{d:.4f}" for w, d in zip(WL, disp)) + "）",
            f"其中平坦 {s_flat:.4f}、波长相关 {s_spec:.4f}；"
            f"波长相关那项代价更大（TOI {tcost[1][1]:.1f} vs 平坦 "
            f"{tcost[0][1]:.1f} %O₂）",
            f"预期 TOI 重复性约 ±{tcost[-1][1]:.1f} %O₂"
            f"（由 {len(codes)} 块估计，样本极少，本身很粗）",
            "k/b 未标定（=1/0）：TOI 的绝对值未经人体或血液体模对标，"
            "只能看趋势和相对变化，不能当绝对血氧读数",
            "体模上的贴合变异未必等于皮肤上的，该外推无证据支持",
        ],
    }
    path.write_text(json.dumps(doc, ensure_ascii=False, indent=2),
                    encoding="utf-8")
    print(f"\n标定文件 → {path}")
    print("  在线运行：python -m srs_cal.online --cal "
          f"\"{path.name}\"")


# ---------------------------------------------------------------------------
def so2_cost(n_mc: int = 4000, sigma_slope: float = 0.01,
             so2_true: float = 70.0, thb_uM: float = 60.0) -> list:
    """同一斜率噪声下，四波长 vs 三波长的 SO₂ 离散。

    只比**相对**放大倍数，所以 sigma_slope 取多少不影响结论 ——
    反演在这个量级上近似线性。
    """
    from fnirs_pipeline.srs import (hemoglobin_extinctions, mua_to_toi,
                                    musp_power_law, slope_to_mueff)
    from srs_cal.forward import predicted_slope as fwd

    rng = np.random.default_rng(0)
    out = []
    for name, wl in (("670/770/810/850（设计）", np.array([670., 770., 810., 850.])),
                     ("670/770/850（若弃 810）", np.array([670., 770., 850.])),
                     ("670/850（最少两路）", np.array([670., 850.]))):
        E = hemoglobin_extinctions(list(map(float, wl)), hw.EXTINCTION_TABLE)
        c = np.array([thb_uM * so2_true / 100, thb_uM * (1 - so2_true / 100)]) * 1e-6
        mua = np.log(10) * (E @ c)
        musp = musp_power_law(wl, hw.MUSP_REF_800, hw.MUSP_EXPONENT)
        s0 = fwd(mua, musp, RHO, hw.N_TISSUE)
        vals = []
        for _ in range(n_mc):
            s = s0 + rng.normal(0, sigma_slope, wl.size)
            m = slope_to_mueff(s, RHO)
            if np.any(m <= 0):
                continue
            t, _ = mua_to_toi(m ** 2 / (3 * musp), wl, hw.EXTINCTION_TABLE)
            if np.isfinite(t):
                vals.append(t)
        out.append((name, float(np.std(vals, ddof=1)), 0.0))
    base = out[0][1]
    return [(n, sd, sd / base) for n, sd, _ in out]


def _toi_at(so2: float, d, wl=WL):
    """给定 SO₂ 真值与斜率扰动 d，反演回来的 TOI。"""
    from fnirs_pipeline.srs import (hemoglobin_extinctions, mua_to_toi,
                                    musp_power_law, slope_to_mueff)
    from srs_cal.forward import predicted_slope as fwd

    E = hemoglobin_extinctions(list(map(float, wl)), hw.EXTINCTION_TABLE)
    musp = musp_power_law(wl, hw.MUSP_REF_800, hw.MUSP_EXPONENT)
    c = np.array([60 * so2 / 100, 60 * (1 - so2 / 100)]) * 1e-6
    m = slope_to_mueff(fwd(np.log(10) * (E @ c), musp, RHO, hw.N_TISSUE) + d, RHO)
    if np.any(m <= 0):
        return np.nan
    return mua_to_toi(m ** 2 / (3 * musp), wl, hw.EXTINCTION_TABLE)[0]


def toi_cost(s_flat: float, s_spec: float, n_mc: int = 4000) -> list:
    """平坦偏移与波长相关残差各自造成多少 TOI 离散。"""
    rng = np.random.default_rng(0)
    out = []
    for name, sf, sw in ((f"只有平坦偏移 σ={s_flat:.4f}", s_flat, 0.0),
                         (f"只有波长相关 σ={s_spec:.4f}", 0.0, s_spec),
                         ("实测的组合", s_flat, s_spec)):
        v = [t for t in (_toi_at(70.0, rng.normal(0, sf)
                                 + rng.normal(0, sw, WL.size))
                         for _ in range(n_mc)) if np.isfinite(t)]
        out.append((name, float(np.std(v, ddof=1))))
    return out


def flat_is_linear(d: float = 0.058) -> tuple[float, float, float]:
    """平坦偏移造成的 TOI 偏差对 SO₂ 是否线性 → 能否被 k/b 吸收。

    返回 (k, b, 线性拟合的最大残差)。残差小 ⇒ 这类误差是 k·TOI+b 形式，
    人体降氧标定能完全吃掉它。
    """
    xs = np.arange(40.0, 95.0, 5.0)
    bias = np.array([_toi_at(s, d) - _toi_at(s, 0.0) for s in xs])
    k, b = np.polyfit(xs, bias, 1)
    resid = float(np.abs(bias - (k * xs + b)).max())
    return 1.0 / (1.0 + k), -b / (1.0 + k), resid


VALIDATION_PROTOCOL = """\
  目标不是「G 拟合得好」—— 单参数拟合 4 个观测必然好看。目标是
  **G 在没参与拟合的块上依然成立**，且 **μa 能回收到认证值**。

  A. 采满 12 块，每块【重复贴合 3 次】（取下再贴，不是连采三段）
     重复贴合是这一步最关键、也最容易被省掉的一项。它分开两件事：
       · 同块 3 次的离散  = 贴合/耦合带来的变异
       · 块间 12 个均值的离散 = 器件 + 模型带来的变异
     若前者 ≈ 后者，那么块间离散根本不是器件问题，是操作重复性问题，
     再怎么调算法都没用。目前 4 块各 1 次，这两项**分不开**。

  B. 一级判据：**把 G 的块间差异拆成「平坦」与「波长相关」两份**
     这一项比 μa 回收率更该先看。实测（4 块）平坦 σ=0.0563、
     波长相关 σ=0.0130，而它们对 TOI 的代价是 1.01 和 1.48 %O₂ ——
     **小的那一项反而更贵**。原因：SO₂ 取的是 μa 的谱比值，对所有
     波长等量的偏移大部分抵消，只有改变谱形的那部分留下来。
     判据：
       · 波长相关 σ < 0.005   （→ TOI 贡献 < 0.6 %O₂）
       · 平坦 σ 不设硬限 —— 它对 SO₂ 的偏差几乎严格线性
         （残差 0.09 %O₂），可被人体 k·TOI+b 吸收，**前提是它在
         标定与测量之间不变**。所以它真正的判据在 A：同块重贴 3 次
         的平坦偏移离散，必须远小于块间的平坦偏移离散。

  B2. 二级判据：μa 回收回归
     把 12 块 × 3 波长 = 36 个点画 反演μa vs 认证μa：
       · 斜率应在 1.00 ± 0.05
       · 截距应 |b| < 0.005 cm⁻¹
       · R² > 0.98
     斜率偏离 1 = 尺度错（μs′ 或 2/ρ̄ 近似）；截距非零 = 加性误差
     （暗电平、杂散光）；R² 低 = 随机性太大。三种病因对应三种修法。
     ⚠️ 这一项对平坦偏移过度敏感（μa∝μeff²，放大 2 倍），所以它
     不合格不等于 TOI 不合格 —— 先看 B，再看这里。

  C. 二级判据：趋势检验（calibrate.py 已实现）
     G 不应随块的 μa 或 μs′ 系统漂移。|t| > 3.448（Bonferroni，
     12 块 2 参数 3 波长）即判不合格 —— 它说明「G 是常数」这个模型
     形式本身错了，而体模的 μa/μs′ 与组织并不重合，这个趋势外推到
     组织上会变成系统偏差。离散检验抓不到它。

  D. 三级判据：留一交叉
     轮流留出 1 块，用其余 11 块定 G 去预测它。预测斜率误差应与
     该块自身的测量不确定度同量级。明显更大 = G 没有外推能力。

  E. 边界检查（尚未做）
     体模 Φ10 cm，而 ρ=4 cm。半无限假设在边缘可能不成立。
     做法：同一块，把探头分别放在正中心和偏离中心 2 cm 处各采一次。
     两者斜率差若 > 测量不确定度，说明有边界效应，4 cm 这一路的
     预测斜率需要改用有限尺寸模型。

  F. 这一版【不能】得出的结论
     · 不能给出可出厂的 G(λ)：4 块、每块 1 次，样本量不够
     · 不能给出 TOI 的绝对准确度：体模不含血红蛋白，测不了
     · 不能确认 k/b：那要靠血液体模降氧曲线或人体对标
"""


def make_figure(codes, xs, ys, k, b, r2, per, disp, gstruct) -> None:
    """三格验证图 → docs/验证图/。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei"]
    plt.rcParams["axes.unicode_minus"] = False
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.6))

    a = ax[0]
    lim = [0, max(xs.max(), ys.max()) * 1.1]
    a.plot(lim, lim, "k--", lw=1, label="理想 y=x")
    a.plot(lim, [k * v + b for v in lim], "-", color="#d62728", lw=1.5,
           label=f"拟合 k={k:.3f} R²={r2:.3f}")
    for j, (w, mk) in enumerate(zip(WL, "os^")):
        a.scatter(xs[j::WL.size], ys[j::WL.size], marker=mk, s=55,
                  label=f"{w:.0f} nm", zorder=3)
    a.set_xlabel("认证 $\mu_a$ (cm$^{-1}$)")
    a.set_ylabel("SRS 反演 $\mu_a$ (cm$^{-1}$)")
    a.set_title("① $\mu_a$ 回收 —— 一级判据")
    a.legend(fontsize=8)
    a.grid(alpha=0.3)

    a = ax[1]
    x = np.arange(WL.size)
    for i, c in enumerate(codes):
        a.plot(x, per[i], "o-", lw=1.2, ms=6, label=c, alpha=0.85)
    a.set_xticks(x)
    a.set_xticklabels([f"{w:.0f}" for w in WL])
    a.set_xlabel("波长 (nm)")
    a.set_ylabel("G(λ)")
    a.set_title(f"② 各块单独反推的 G —— 离散 {disp.mean():.3f}")
    a.legend(fontsize=8)
    a.grid(alpha=0.3)

    a = ax[2]
    tc = toi_cost(*gstruct)
    names = ["平坦偏移\nσ=%.4f" % gstruct[0], "波长相关\nσ=%.4f" % gstruct[1],
             "两者合计"]
    a.bar(range(3), [c[1] for c in tc], color=["#aab7c4", "#d62728", "#444444"])
    for i, c in enumerate(tc):
        a.text(i, c[1], f"{c[1]:.2f}", ha="center", va="bottom",
               fontsize=10, fontweight="bold")
    a.set_xticks(range(3))
    a.set_xticklabels(names, fontsize=8)
    a.set_ylabel("TOI 标准差 (%)")
    a.set_title("③ 哪一半才贵 —— 小的那项更贵")
    a.grid(alpha=0.3, axis="y")

    fig.suptitle("SRS v1（" + "/".join(f"{w:.0f}" for w in WL)
                 + "）—— 真实体模数据验证",
                 fontsize=13, fontweight="bold")
    fig.tight_layout()
    DOCS_FIG.mkdir(parents=True, exist_ok=True)
    out = DOCS_FIG / "SRS_v1_体模验证.png"
    fig.savefig(out, dpi=140)
    print(f"\n验证图 → {out}")


if __name__ == "__main__":
    raise SystemExit(main())
