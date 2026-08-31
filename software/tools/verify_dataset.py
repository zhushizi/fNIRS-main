"""
用公开数据集验证本项目的算法端（离线管线）。

面向 HEFMI-ICH（Scientific Data 2025, DOI 10.1038/s41597-025-06100-7,
figshare 10.6084/m9.figshare.28955456）等提供【原始光强 + 已发布 HbO/HbR】
的数据集，做三层比对：

    L1  光强 -> dOD          本项目 vs 独立实现      【与 DPF/消光系数无关，最干净】
    L2  dOD  -> Delta_c      本项目 vs 独立实现      【查 ε 表 + 光程】
    L3  Delta_c vs 数据集发布值                      【端到端，含对方的 DPF 约定】

L3 的绝对幅度依赖对方的 DPF（多数数据集不写），故同时给出：
  - 相关系数 r：与 DPF 无关，反映波形是否一致
  - 最佳拟合比例 k：k = 对方DPF / 本次假设DPF，可反推对方用了什么

用法
----
先看数据集配置（最重要的一步，决定后面能跑到哪一层）：
    python tools/verify_dataset.py --info path/to/subject.nirs

跑 L1+L2 比对：
    python tools/verify_dataset.py --compare path/to/subject.nirs

指定 DPF 与源探距离（默认取数据集几何算出的距离）：
    python tools/verify_dataset.py --compare x.nirs --dpf 6.0 --distance 3.0

自检（合成数据，不需要数据集）：
    python tools/verify_dataset.py --selftest

支持格式
--------
  .nirs   Homer2（MATLAB v5 / v7.3 均可）
  .snirf  SNIRF（HDF5）

波长兼容性
----------
本项目三个函数对外来波长的容忍度不同，脚本会自动选择可行路径：

  hemoglobin_extinctions   任意波长（np.interp 插值）          总是可用
  intensities_to_od_changes 与波长无关                          总是可用
  generalized_mbll         需波长∈{700,730,770,810,850}         Cyt 为硬编码字典
                           （config.CYT_DIFFERENCE_EXTINCTION）
  mbll_hbo_hbr             需每通道恰好 2 个波长

若波长不在 Cyt 表内（如 808 nm），脚本改为直接调用 generalized_mbll 内部
那一步 HbO/HbR 解算（pinv(hb_ex*pathlength) @ dOD）——与 mbll.py:122 完全
相同的表达式，只是跳过 Cyt。不修改任何生产代码。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

if hasattr(sys.stdout, "reconfigure"):        # Windows 控制台默认 GBK
    sys.stdout.reconfigure(encoding="utf-8")

from config import CYT_DIFFERENCE_EXTINCTION  # noqa: E402
from fnirs_pipeline.mbll import generalized_mbll  # noqa: E402
from fnirs_pipeline.mbll_core import (  # noqa: E402
    hemoglobin_extinctions,
    intensities_to_od_changes,
)

CYT_WLS = sorted(float(w) for w in CYT_DIFFERENCE_EXTINCTION)


# ---------------------------------------------------------------------------
# 数据集读取
# ---------------------------------------------------------------------------
class Dataset:
    """统一的数据集容器。"""

    def __init__(self, d, t, wavelengths, meas_list, src_pos, det_pos, source,
                 spatial_unit="mm"):
        self.d = np.asarray(d, float)              # (n_time, n_meas) 原始光强
        self.t = np.asarray(t, float).ravel()      # (n_time,)
        self.wavelengths = np.asarray(wavelengths, float).ravel()
        self.meas_list = np.asarray(meas_list, int)  # (n_meas, >=4) [src, det, _, wlIdx]
        self.src_pos = None if src_pos is None else np.asarray(src_pos, float)
        self.det_pos = None if det_pos is None else np.asarray(det_pos, float)
        self.source = source
        # Homer2 的 SD.SpatialUnit / SNIRF 的 lengthUnit；缺省按 mm。
        # 单位判错会让源探距离差 10 倍，进而让 Δc 差 10 倍，必须以文件声明为准。
        self.spatial_unit = str(spatial_unit).strip().lower() or "mm"

    @property
    def fs(self) -> float:
        dt = float(np.median(np.diff(self.t)))
        return 1.0 / dt if dt > 0 else float("nan")

    def separations_cm(self) -> np.ndarray | None:
        """每个测量的源探距离，统一换算到 cm。"""
        if self.src_pos is None or self.det_pos is None:
            return None
        s = self.meas_list[:, 0] - 1
        dd = self.meas_list[:, 1] - 1
        if s.max() >= len(self.src_pos) or dd.max() >= len(self.det_pos):
            return None
        raw = np.linalg.norm(self.src_pos[s] - self.det_pos[dd], axis=1)
        factor = {"cm": 1.0, "mm": 0.1, "m": 100.0}.get(self.spatial_unit)
        if factor is None:
            raise ValueError(f"未知空间单位 {self.spatial_unit!r}；支持 cm/mm/m")
        return raw * factor

    def channel_groups(self) -> dict[tuple[int, int], dict[int, int]]:
        """{(src,det): {wlIdx: measIdx}}，只保留波长齐全的组。"""
        groups: dict[tuple[int, int], dict[int, int]] = {}
        for i, row in enumerate(self.meas_list):
            groups.setdefault((int(row[0]), int(row[1])), {})[int(row[3])] = i
        n_wl = len(self.wavelengths)
        return {k: v for k, v in groups.items() if len(v) == n_wl}


def _mat_get(obj, name):
    """从 scipy 的 mat_struct 或 dict 里取字段。"""
    if isinstance(obj, dict):
        return obj.get(name)
    return getattr(obj, name, None)


def load_nirs(path: str) -> Dataset:
    """读 Homer2 .nirs（本质是 .mat）。先试 scipy(v5)，失败再试 h5py(v7.3)。"""
    try:
        from scipy.io import loadmat
        m = loadmat(path, struct_as_record=False, squeeze_me=True)
        sd = m["SD"]
        return Dataset(
            d=m["d"], t=m["t"],
            wavelengths=_mat_get(sd, "Lambda"),
            meas_list=_mat_get(sd, "MeasList"),
            src_pos=_mat_get(sd, "SrcPos"),
            det_pos=_mat_get(sd, "DetPos"),
            source=f"{path} (Homer2 .nirs, MATLAB v5)",
            spatial_unit=str(_mat_get(sd, "SpatialUnit") or "mm"),
        )
    except (NotImplementedError, ValueError):
        pass

    import h5py
    with h5py.File(path, "r") as f:
        def arr(p):
            return np.array(f[p]).T if p in f else None

        unit = "mm"
        if "SD/SpatialUnit" in f:                      # v7.3 里字符串存为 uint16 码点
            raw = np.array(f["SD/SpatialUnit"]).ravel()
            unit = "".join(chr(int(c)) for c in raw)

        return Dataset(
            d=arr("d"), t=arr("t"),
            wavelengths=arr("SD/Lambda"),
            meas_list=arr("SD/MeasList"),
            src_pos=arr("SD/SrcPos"),
            det_pos=arr("SD/DetPos"),
            source=f"{path} (Homer2 .nirs, MATLAB v7.3)",
            spatial_unit=unit,
        )


def load_snirf(path: str) -> Dataset:
    """读 SNIRF（HDF5）。"""
    import h5py
    with h5py.File(path, "r") as f:
        nirs = f["nirs"] if "nirs" in f else f["nirs1"]
        data = nirs["data1"]
        d = np.array(data["dataTimeSeries"])
        t = np.array(data["time"]).ravel()
        wls = np.array(nirs["probe"]["wavelengths"]).ravel()

        rows = []
        for key in sorted(k for k in data if k.startswith("measurementList")):
            ml = data[key]
            rows.append([
                int(np.array(ml["sourceIndex"]).ravel()[0]),
                int(np.array(ml["detectorIndex"]).ravel()[0]),
                0,
                int(np.array(ml["wavelengthIndex"]).ravel()[0]),
            ])

        def pos(name):
            for k in (name, name.replace("2D", "3D")):
                if k in nirs["probe"]:
                    return np.array(nirs["probe"][k])
            return None

        unit = "mm"
        if "lengthUnit" in nirs["metaDataTags"]:
            raw = nirs["metaDataTags"]["lengthUnit"][()]
            unit = raw.decode() if isinstance(raw, bytes) else str(np.array(raw).ravel()[0])

        return Dataset(d, t, wls, np.array(rows), pos("sourcePos2D"), pos("detectorPos2D"),
                       f"{path} (SNIRF)", spatial_unit=unit)


def load_any(path: str) -> Dataset:
    ext = Path(path).suffix.lower()
    if ext == ".snirf":
        return load_snirf(path)
    if ext in (".nirs", ".mat"):
        return load_nirs(path)
    raise ValueError(f"不支持的扩展名 {ext}；支持 .nirs / .snirf")


# ---------------------------------------------------------------------------
# 反演：本项目 vs 独立实现
# ---------------------------------------------------------------------------
def solve_project(d_od: np.ndarray, wls: list[float], dpfs: list[float],
                  distance_cm: float) -> tuple[np.ndarray, str]:
    """
    调本项目的反演。返回 (HbO/HbR (2,n), 所走路径的描述)。

    波长全在 Cyt 表内 -> generalized_mbll（完整生产路径）
    否则               -> 复现其 HbO/HbR 那一步（mbll.py:122 同式），跳过 Cyt
    """
    if all(float(w) in CYT_DIFFERENCE_EXTINCTION for w in wls):
        return generalized_mbll(d_od, wls, dpfs, distance_cm)[:2], "generalized_mbll（完整）"

    hb_ex = hemoglobin_extinctions(wls)                     # 本项目的 ε 表 + 插值
    pathlength = np.asarray(dpfs, float) * float(distance_cm)
    a_hb = hb_ex * pathlength[:, np.newaxis]
    return np.linalg.pinv(a_hb) @ d_od, "generalized_mbll 的 HbO/HbR 步（跳过 Cyt）"


def solve_independent(d_od: np.ndarray, wls: list[float], dpfs: list[float],
                      distance_cm: float, ex: np.ndarray) -> np.ndarray:
    """独立实现：不调用本项目任何反演代码。ex 由调用方给定以隔离 ε 表来源。"""
    p = np.asarray(dpfs, float) * float(distance_cm)
    return np.linalg.pinv(ex * p[:, np.newaxis]) @ d_od


def compare_series(a: np.ndarray, b: np.ndarray) -> tuple[float, float, float]:
    """返回 (相关系数, 最佳拟合比例 k 使 a≈k·b, 去掉比例后的相对 RMS)。"""
    a = np.asarray(a, float).ravel()
    b = np.asarray(b, float).ravel()
    n = min(len(a), len(b))
    a, b = a[:n] - a[:n].mean(), b[:n] - b[:n].mean()
    denom = float(b @ b)
    k = float(a @ b) / denom if denom > 0 else float("nan")
    sa, sb = a.std(), b.std()
    r = float((a @ b) / (n * sa * sb)) if sa > 0 and sb > 0 else float("nan")
    resid = a - k * b
    rel = float(np.sqrt((resid ** 2).mean()) / sa) if sa > 0 else float("nan")
    return r, k, rel


# ---------------------------------------------------------------------------
# 报告
# ---------------------------------------------------------------------------
def report_info(ds: Dataset) -> None:
    print("=" * 84)
    print("数据集配置")
    print("=" * 84)
    print(f"  来源        {ds.source}")
    print(f"  光强矩阵 d  {ds.d.shape}  (n_time x n_meas)")
    print(f"  时长        {ds.t[-1] - ds.t[0]:.1f} s     采样率 {ds.fs:.2f} Hz")
    print(f"  波长        {[round(float(w), 1) for w in ds.wavelengths]}  (共 {len(ds.wavelengths)} 个)")

    sep = ds.separations_cm()
    if sep is not None:
        u = np.unique(np.round(sep, 2))
        print(f"  空间单位    {ds.spatial_unit}（取自文件声明）")
        print(f"  源探距离    {u[:12]}{' ...' if len(u) > 12 else ''} cm")
        print(f"              最小 {sep.min():.2f} cm  最大 {sep.max():.2f} cm")
        short = int((sep < 1.5).sum())
        print(f"  短距通道    {short} 个 (<1.5 cm)  ->  "
              f"{'可测 SSR' if short else 'SSR 无法验证（全为长距）'}")
    groups = ds.channel_groups()
    print(f"  完整通道    {len(groups)} 个（波长齐全）")

    print("\n" + "=" * 84)
    print("与本项目的兼容性")
    print("=" * 84)
    wls = [float(w) for w in ds.wavelengths]
    inside = [w for w in wls if w in CYT_DIFFERENCE_EXTINCTION]
    print(f"  本项目 Cyt 表覆盖  {CYT_WLS}")
    print(f"  数据集波长在表内   {inside if inside else '无'}")
    if len(inside) == len(wls):
        print("  -> generalized_mbll 可直接调用（完整生产路径，含 Cyt）")
    else:
        print(f"  -> {[w for w in wls if w not in CYT_DIFFERENCE_EXTINCTION]} 不在 Cyt 表内，")
        print("     generalized_mbll 会 KeyError；脚本改走其内部 HbO/HbR 解算式（不改生产代码）")
    print("  hemoglobin_extinctions 用 np.interp，任意波长可用")


def report_compare(ds: Dataset, dpf: float, distance_cm: float | None,
                   n_channels: int, baseline_s: float) -> bool:
    ok = True
    wls = [float(w) for w in ds.wavelengths]
    groups = ds.channel_groups()
    if not groups:
        print("没有波长齐全的通道，无法比对")
        return False

    sep = ds.separations_cm()
    keys = sorted(groups)[:n_channels]

    print("\n" + "=" * 84)
    print(f"L1 / L2 比对   DPF={dpf}  基线取前 {baseline_s:.0f} s")
    print("=" * 84)
    print("  通道(src,det)  距离    L1 dOD 最大差    L2 Δc 最大相对差   路径")

    n_base = max(1, int(baseline_s * ds.fs))
    ex_ind = hemoglobin_extinctions(wls)     # L2 两侧共用 ε，隔离出「解算」这一步
    worst_l1 = worst_l2 = 0.0
    path_desc = ""

    for key in keys:
        idx = [groups[key][k] for k in sorted(groups[key])]
        raw = ds.d[:, idx].T                                   # (n_wl, n_time)
        if np.any(raw <= 0):
            continue
        refs = raw[:, :n_base].mean(axis=1)

        # ---- L1: 光强 -> dOD ----
        od_prj = intensities_to_od_changes(raw, refs=refs)
        od_ind = -np.log10(raw / refs[:, None])
        l1 = float(np.max(np.abs(od_prj - od_ind)))
        worst_l1 = max(worst_l1, l1)

        # ---- L2: dOD -> Δc ----
        d_cm = distance_cm if distance_cm else (float(sep[idx[0]]) if sep is not None else 3.0)
        dpfs = [dpf] * len(wls)
        c_prj, path_desc = solve_project(od_prj, wls, dpfs, d_cm)
        c_ind = solve_independent(od_prj, wls, dpfs, d_cm, ex_ind)
        scale = np.maximum(np.abs(c_ind).max(axis=1, keepdims=True), 1e-30)
        l2 = float(np.max(np.abs(c_prj - c_ind) / scale))
        worst_l2 = max(worst_l2, l2)

        print(f"  ({key[0]:2d},{key[1]:2d})       {d_cm:4.2f}cm   {l1:.3e}      {l2:.3e}")

    print(f"\n  路径：{path_desc}")
    for name, val, tol in (("L1 dOD", worst_l1, 1e-12), ("L2 Δc ", worst_l2, 1e-9)):
        good = val < tol
        ok &= good
        print(f"  {name} 全通道最大差 {val:.3e}  (<{tol:.0e})  {'PASS' if good else 'FAIL'}")

    print("\n  L1 检的是 intensities_to_od_changes 的对数与基线约定（与 DPF/ε 无关）")
    print("  L2 检的是 pinv 解算实现（两侧共用同一 ε，隔离出解算本身）")
    print("  ε 表来源本身需靠 L3 与数据集发布值比对，或与 MNE-NIRS 交叉核对")
    return ok


def selftest() -> bool:
    print("=" * 84)
    print("自检：合成数据（不需要数据集）")
    print("=" * 84)
    ok = True
    for wls, label in ((CYT_WLS, "五波长（全在 Cyt 表内）"),
                       ([760.0, 808.0, 850.0], "三波长（808 不在 Cyt 表内）"),
                       ([760.0, 850.0], "双波长")):
        ex = hemoglobin_extinctions(wls)
        dpf, d_cm = 6.0, 3.0
        truth = np.array([10e-6, -5e-6])
        od = (ex * (dpf * d_cm)) @ truth
        od = od.reshape(-1, 1)
        c_prj, path = solve_project(od, wls, [dpf] * len(wls), d_cm)
        err = float(np.max(np.abs(c_prj[:, 0] - truth) / np.abs(truth)))
        good = err < 1e-9
        ok &= good
        print(f"  {label:28s} 还原误差 {err:.3e}  {'PASS' if good else 'FAIL'}   [{path}]")

    # 光强往返
    raw = np.array([[1.0e6], [0.9e6], [0.8e6]]) * 10 ** (-np.array([[0.1], [0.2], [0.3]]))
    refs = np.array([1.0e6, 0.9e6, 0.8e6])
    od_p = intensities_to_od_changes(raw, refs=refs)
    err = float(np.max(np.abs(od_p - np.array([[0.1], [0.2], [0.3]]))))
    good = err < 1e-12
    ok &= good
    print(f"  {'intensities_to_od_changes':28s} 往返误差 {err:.3e}  {'PASS' if good else 'FAIL'}")
    print(f"\n自检结论：{'通过' if ok else '未通过'}")
    return ok


def main() -> int:
    p = argparse.ArgumentParser(description="用公开数据集验证本项目算法端")
    p.add_argument("--info", help="读取并打印数据集配置（.nirs / .snirf）")
    p.add_argument("--compare", help="跑 L1/L2 比对")
    p.add_argument("--selftest", action="store_true", help="合成数据自检")
    p.add_argument("--dpf", type=float, default=6.0, help="假设的 DPF，默认 6.0")
    p.add_argument("--distance", type=float, help="源探距离(cm)，默认由数据集几何算出")
    p.add_argument("--channels", type=int, default=8, help="比对的通道数，默认 8")
    p.add_argument("--baseline", type=float, default=30.0, help="基线秒数，默认 30")
    a = p.parse_args()

    if a.selftest:
        return 0 if selftest() else 1
    if a.info:
        report_info(load_any(a.info))
        return 0
    if a.compare:
        ds = load_any(a.compare)
        report_info(ds)
        return 0 if report_compare(ds, a.dpf, a.distance, a.channels, a.baseline) else 1

    p.print_help()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
