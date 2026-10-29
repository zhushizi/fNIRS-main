# -*- coding: utf-8 -*-
"""原始帧归档 —— 把串口来的字节原样落盘，供事后重放。

为什么需要（架构文档 R9）：
    `online.py` 此前**完全不存数据**，实时监护出了异常无从回看。
    而「两条解码链可分别调试」的物理前提就是可重放 —— 改了降噪想看效果，
    没有归档就只能重新采，可受试者已经动了，比的是两次不同的测量。

为什么存**原始字节**而不是解析后的字段：
    L1 的解析逻辑将来会改（加第二光源、厂家启用 14 字节保留区、改帧长）。
    存解析结果会把**当时解析器的 bug 永久烙进归档**；存原始字节则可以
    用新解析器重跑旧数据，顺带还检验了解析器。

文件格式（自描述，不依赖本模块也能读）：

    第 1 行   UTF-8 的 JSON + "\\n"   —— 文件头，含版本/起始时刻/记录长度
    其后      定长记录，每条 REC_LEN 字节：
                  [0:8]   t_rel  相对文件起始的秒数，float64 小端
                  [8:34]  raw    原始帧字节（FRAME_LEN = 26）

    定长记录让**截断**（掉电、拔盘）无损可读：按 REC_LEN 切，
    最后一条不完整就丢掉，前面的全部有效。

⚠️ **归档失败绝不能中断测量。** 写盘出错（满盘、无权限、拔盘）一律吞掉并
   计入 `n_errors`，由调用方决定是否提示。测量优先于记录，这是医疗设备的
   基本取舍 —— 宁可丢记录，不可丢监护。
"""

from __future__ import annotations

import json
import struct
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

_HERE = Path(__file__).resolve().parent
if str(_HERE.parent) not in sys.path:
    sys.path.insert(0, str(_HERE.parent))

from srs_cal.protocol import (FRAME_LEN, FRAMES_PER_CYCLE,  # noqa: E402
                              Frame, FrameStream)

MAGIC = "SRSRAW"
FORMAT_VERSION = 1
T_LEN = 8                                   # float64
REC_LEN = T_LEN + FRAME_LEN                 # 34
_REC = struct.Struct("<d")

DEFAULT_DIR = _HERE / "原始归档"
DEFAULT_ROTATE_S = 3600.0                   # 每小时一个文件
DEFAULT_MAX_BYTES = 5 * 1024 ** 3           # 总量上限 5 GB，超了删最旧
SUFFIX = ".srsraw"


# ---------------------------------------------------------------------------
# 写
# ---------------------------------------------------------------------------
@dataclass
class ArchiveStats:
    files: int = 0
    frames: int = 0
    bytes: int = 0
    n_errors: int = 0
    last_error: str = ""
    purged_files: int = 0
    purged_bytes: int = 0


class RawArchive:
    """追加写的原始帧归档，按时间轮转，总量超限删最旧。

    用法：
        ar = RawArchive(meta={"port": "COM16", "baud": 115200})
        ar.write(frames)        # frames 为 protocol.Frame 列表
        ar.close()

    线程安全性：**不保证**。调用方自行保证只在一个线程里写
    （online.py 里所有帧都在 GUI 线程的槽函数里处理，天然满足）。
    """

    def __init__(self, directory: Path | str = DEFAULT_DIR,
                 rotate_s: float = DEFAULT_ROTATE_S,
                 max_bytes: int = DEFAULT_MAX_BYTES,
                 meta: dict | None = None,
                 clock=time.perf_counter) -> None:
        self.dir = Path(directory)
        self.rotate_s = float(rotate_s)
        self.max_bytes = int(max_bytes)
        self.meta = dict(meta or {})
        self._clock = clock
        self.stats = ArchiveStats()
        self._fh = None
        self._path: Path | None = None
        self._t0 = 0.0                      # 当前文件的 t_host 起点
        self._opened_at = 0.0               # 当前文件的开启时刻（用于轮转）
        self.enabled = True

    # -- 文件管理 ----------------------------------------------------------
    def _header(self) -> bytes:
        h = {
            "magic": MAGIC,
            "version": FORMAT_VERSION,
            "created": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "t0_wall": time.time(),          # 墙钟，用于和真实事件对时
            "t0_perf": self._t0,             # 对应的 perf_counter，用于还原 t_host
            "rec_len": REC_LEN,
            "frame_len": FRAME_LEN,
            # **单个通道**一周期的帧数。多通道同采时串口上一周期是它的
            # 通道数倍 —— 读归档时别拿这个数去除总帧数。
            "frames_per_cycle": FRAMES_PER_CYCLE,
            "note": "t_rel = float64 小端，相对 t0_perf 的秒数；其后 raw 帧字节",
            **self.meta,
        }
        return (json.dumps(h, ensure_ascii=False) + "\n").encode("utf-8")

    def _open(self, t_host: float) -> None:
        self.dir.mkdir(parents=True, exist_ok=True)
        # ⚠️ 文件名必须唯一。只用到秒的时间戳在**同一秒内轮转两次**时会
        #    静默覆盖前一个文件（2026-10-22 自检抓到）。按小时轮转时不易
        #    撞上，但程序快速重启、或将来改成按大小轮转就会丢数据 ——
        #    归档丢数据是无声的，必须在这一层堵死。
        base = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._path = self.dir / (base + SUFFIX)
        n = 0
        while self._path.exists():
            n += 1
            self._path = self.dir / f"{base}_{n:03d}{SUFFIX}"
        self._t0 = t_host
        self._fh = self._path.open("wb")
        self._fh.write(self._header())
        self._fh.flush()
        self._opened_at = self._clock()
        self.stats.files += 1

    def _close_fh(self) -> None:
        if self._fh is not None:
            try:
                self._fh.flush()
                self._fh.close()
            except Exception:                            # noqa: BLE001
                pass
        self._fh = None

    def _purge(self) -> None:
        """总量超限时删最旧的文件。当前正在写的那个永不删。"""
        files = sorted(self.dir.glob("*" + SUFFIX))
        total = sum(f.stat().st_size for f in files if f.exists())
        for f in files:
            if total <= self.max_bytes:
                break
            if self._path is not None and f == self._path:
                continue
            try:
                n = f.stat().st_size
                f.unlink()
                total -= n
                self.stats.purged_files += 1
                self.stats.purged_bytes += n
            except Exception:                            # noqa: BLE001
                break

    # -- 写入 --------------------------------------------------------------
    def write(self, frames) -> int:
        """写一批帧，返回成功写入的条数。出错不抛，计入 stats.n_errors。"""
        if not self.enabled or not frames:
            return 0
        try:
            t_first = float(frames[0].t_host)
            if self._fh is None:
                self._open(t_first)
            elif self._clock() - self._opened_at >= self.rotate_s:
                self._close_fh()
                self._purge()
                self._open(t_first)

            buf = bytearray()
            for f in frames:
                raw = f.raw
                if len(raw) != FRAME_LEN:        # 不该发生；跳过而不是写坏
                    continue
                buf += _REC.pack(float(f.t_host) - self._t0) + raw
            if not buf:
                return 0
            self._fh.write(buf)
            self._fh.flush()                     # 掉电时最多丢一批
            n = len(buf) // REC_LEN
            self.stats.frames += n
            self.stats.bytes += len(buf)
            return n
        except Exception as e:                           # noqa: BLE001
            # ⚠️ 测量优先：归档出错不得中断监护
            self.stats.n_errors += 1
            self.stats.last_error = f"{type(e).__name__}: {e}"
            self._close_fh()
            return 0

    def close(self) -> None:
        self._close_fh()
        try:
            self._purge()
        except Exception:                                # noqa: BLE001
            pass

    @property
    def current_path(self) -> Path | None:
        return self._path if self._fh is not None else None

    def summary(self) -> str:
        s = self.stats
        mb = s.bytes / 1024 ** 2
        out = f"{s.frames:,} 帧 / {mb:.1f} MB / {s.files} 文件"
        if s.purged_files:
            out += f"（已清理 {s.purged_files} 个旧文件）"
        if s.n_errors:
            out += f"　⚠ 写入错误 {s.n_errors} 次：{s.last_error}"
        return out


# ---------------------------------------------------------------------------
# 读 / 重放
# ---------------------------------------------------------------------------
def read_header(path: Path | str) -> dict:
    with Path(path).open("rb") as fh:
        line = fh.readline()
    h = json.loads(line.decode("utf-8"))
    if h.get("magic") != MAGIC:
        raise ValueError(f"不是 {MAGIC} 归档：{path}")
    return h


def read_records(path: Path | str):
    """逐条产出 (t_rel, raw_bytes)。末尾不完整的记录自动丢弃。"""
    p = Path(path)
    with p.open("rb") as fh:
        fh.readline()                        # 跳过头
        rec_len = REC_LEN
        while True:
            blk = fh.read(rec_len)
            if len(blk) < rec_len:           # 截断（掉电/拔盘）→ 到此为止
                return
            yield _REC.unpack_from(blk, 0)[0], blk[T_LEN:]


def replay(path: Path | str, channel: int | None = None) -> list[Frame]:
    """把归档喂回 FrameStream，还原出带 t_host / cycle_id 的 Frame 列表。

    ⚠️ 这里刻意**重新解析**而不是直接反序列化字段 —— 这样归档既是数据，
       也是解析器的回归测试集。协议改了用新解析器重跑即可。

    `channel` 给了就只返回该采集通道的帧。

    ⚠️⚠️ **多通道归档必须筛通道。** 一份双通道归档里两个通道的帧是
       交错的，而 `DarkSubtractor` 只认 (λ, ρ)、不认通道 —— 不筛就会把
       另一个通道的暗时隙扣到这个通道的信号上。2026-10-29 实测 2 号通道
       暗时隙峰峰 786,447，而 1 号只有 11,461，混着扣出来的净值毫无意义。
       在线那两条链（online / acquire）安全是因为它们在处理前就按 byte10
       过滤过，离线没有那道闸 —— 这个参数就是那道闸。
       默认 `None`（不筛）是为了兼容既有调用方，**新代码请显式传通道**。
    """
    h = read_header(path)
    t0 = float(h.get("t0_perf", 0.0))
    st = FrameStream()
    out: list[Frame] = []
    for t_rel, raw in read_records(path):
        out.extend(st.feed(raw, t_host=t0 + t_rel))
    if channel is not None:
        out = [f for f in out if f.channel == int(channel)]
    return out


def replay_channels(path: Path | str) -> dict[int, list[Frame]]:
    """同 `replay`，但按采集通道分好组：{通道号: [该通道的帧]}。

    通道**从回传推断**（数据帧 byte10），不读归档头里下发过什么 ——
    下发了不等于设备开了（2026-10-28 实测：下发 2 号，292 帧回来全是 01）。
    """
    out: dict[int, list[Frame]] = {}
    for f in replay(path):
        out.setdefault(int(f.channel), []).append(f)
    return out


def list_archives(directory: Path | str = DEFAULT_DIR) -> list[Path]:
    d = Path(directory)
    return sorted(d.glob("*" + SUFFIX)) if d.exists() else []
