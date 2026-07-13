"""
解码器统一日志：控制台 + 持久化滚动文件。

设计目标：安卓端运行时 `print` 不易抓取，这里改用标准 logging 落地到文件，
方便事后取回排查。

日志目录优先级：
1. `configure_logging(log_dir=...)` 显式传入（安卓启动时应传 app 可写目录，
   如 `context.getFilesDir()`，因为 online_android 目录在 APK 内通常不可写）；
2. 环境变量 `FNIRS_LOG_DIR`；
3. 默认 `online_android/logs/`（PC/开发用）。

特性：
- 滚动文件（默认 2MB × 5 份），UTF-8（中文日志不乱码）；
- 线程安全（worker 线程与 TCP rx 线程并发写，logging 自身加锁）；
- 目录不可写时**自动退化为仅控制台，不抛异常**（安卓只读目录也不会崩）；
- 懒初始化：任何 `get_logger()` 首次调用会用默认配置自动建好。

典型用法：
    from online_android.logging_utils import configure_logging, get_logger
    configure_logging(log_dir="/data/data/<pkg>/files/fnirs_logs")  # 安卓启动时
    log = get_logger("worker")
    log.info("online started")
"""

from __future__ import annotations

import logging
import os
import threading
from logging.handlers import RotatingFileHandler
from pathlib import Path

_PACKAGE_DIR = Path(__file__).resolve().parent
_DEFAULT_LOG_DIR = _PACKAGE_DIR / "logs"
_BASE_LOGGER = "fnirs"
_LOG_FILENAME = "fnirs_decoder.log"
_ENV_LOG_DIR = "FNIRS_LOG_DIR"

_lock = threading.Lock()
_configured = False
_log_file_path: Path | None = None

_FORMAT = "%(asctime)s %(levelname)s [%(name)s] (%(threadName)s) %(message)s"
_DATEFMT = "%Y-%m-%d %H:%M:%S"

# 导入时给基础 logger 挂一个 NullHandler 并关闭向 root 冒泡：
# 这样在 configure_logging() 被调用前，模块级 get_logger() 拿到的 logger 不会
# 报 "No handlers"、也不会把日志漏到 root/stderr。真正的 handler 由
# configure_logging() 装上——安卓可在启动时先指定可写目录，PC 由入口自动配置。
_base_logger = logging.getLogger(_BASE_LOGGER)
_base_logger.addHandler(logging.NullHandler())
_base_logger.propagate = False


def _resolve_log_dir(log_dir: str | os.PathLike | None) -> Path:
    if log_dir is not None:
        return Path(log_dir)
    env = os.environ.get(_ENV_LOG_DIR)
    if env:
        return Path(env)
    return _DEFAULT_LOG_DIR


def configure_logging(
    log_dir: str | os.PathLike | None = None,
    *,
    level: int = logging.INFO,
    to_console: bool = True,
    max_bytes: int = 2_000_000,
    backup_count: int = 5,
    force: bool = False,
) -> logging.Logger:
    """配置解码器日志（幂等）。返回基础 logger `fnirs`。

    安卓端应在启动时调用一次并传入可写的 `log_dir`。重复调用默认不生效，
    传 `force=True` 可用新参数（如新目录）重配。
    """
    global _configured, _log_file_path
    with _lock:
        base = logging.getLogger(_BASE_LOGGER)
        if _configured and not force:
            return base

        for handler in list(base.handlers):
            base.removeHandler(handler)
            try:
                handler.close()
            except Exception:
                pass

        base.setLevel(level)
        base.propagate = False  # 独立 logger，不往 root 冒泡，避免和第三方日志混串
        formatter = logging.Formatter(_FORMAT, datefmt=_DATEFMT)

        if to_console:
            console = logging.StreamHandler()
            console.setFormatter(formatter)
            base.addHandler(console)

        _log_file_path = None
        target_dir = _resolve_log_dir(log_dir)
        try:
            target_dir.mkdir(parents=True, exist_ok=True)
            file_path = target_dir / _LOG_FILENAME
            file_handler = RotatingFileHandler(
                str(file_path),
                maxBytes=max_bytes,
                backupCount=backup_count,
                encoding="utf-8",
            )
            file_handler.setFormatter(formatter)
            base.addHandler(file_handler)
            _log_file_path = file_path
        except Exception as exc:
            # 目录不可写（如安卓 APK 内只读路径）→ 仅控制台，绝不因日志把主流程搞崩。
            base.warning("File logging disabled (cannot write to %s): %s", target_dir, exc)

        if not base.handlers:
            # 文件失败且未开控制台 → 补一个 NullHandler，避免无 handler 告警。
            base.addHandler(logging.NullHandler())

        _configured = True
        if _log_file_path is not None:
            base.info("Decoder logging to file: %s", _log_file_path)
        return base


def get_logger(name: str | None = None) -> logging.Logger:
    """
    取一个命名 logger（`fnirs.<name>`）。

    注意：此函数**不会**自动配置日志——configure_logging() 未调用前，日志会被
    NullHandler 静默吞掉（不崩、不漏到 stderr）。请在程序入口显式调用一次
    configure_logging()（PC 入口 run_pipeline 已自动调用；安卓在启动时调用并传
    可写目录）。模块级先 get_logger() 再稍后 configure_logging() 是安全的：handler
    挂在基础 logger 上，子 logger 通过传播使用。
    """
    if not name:
        return logging.getLogger(_BASE_LOGGER)
    return logging.getLogger(f"{_BASE_LOGGER}.{name}")


def log_file_path() -> str | None:
    """当前日志文件的绝对路径；仅控制台（文件被禁用）时返回 None。"""
    return str(_log_file_path) if _log_file_path is not None else None
