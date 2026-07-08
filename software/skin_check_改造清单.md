# 皮肤检查（佩戴质量检测）改造清单

> **文档版本**：2026-07-06 v2  
> **适用仓库**：`fNIRS-main/software`  
> **读者**：PC 开发、安卓开发、测试、产品  

---

## 目录

1. [这份文档解决什么问题](#1-这份文档解决什么问题)
2. [和皮肤检查、VOT、正式采集的区别](#2-和皮肤检查vot正式采集的区别)
3. [推荐方案总览](#3-推荐方案总览)
4. [架构：要不要单独做模块](#4-架构要不要单独做模块)
5. [用户流程](#5-用户流程)
6. [检测算法（三层组合）](#6-检测算法三层组合)
7. [TCP 协议扩展](#7-tcp-协议扩展)
8. [PC 端改造清单](#8-pc-端改造清单)
9. [安卓端改造清单](#9-安卓端改造清单)
10. [测试清单](#10-测试清单)
11. [阈值标定](#11-阈值标定)
12. [分阶段计划](#12-分阶段计划)
13. [FAQ](#13-faq)
14. [相关文件索引](#14-相关文件索引)

---

## 1. 这份文档解决什么问题

当前主链路 `fNIRS_processing.py` → `run_pipeline()` 只做：

```text
采集原始光强 → 预处理 → MBLL/SSR → 在线回传 HbO/HbR/Cyt/rSO₂ → 离线落盘
```

**没有**「戴好了没有」的独立步骤。用户贴探头常见问题：

| 现象 | 后果 |
|------|------|
| 探头未贴紧 | 光强低、波动大，MBLL 不可靠 |
| 头发遮挡 | 某路波长无数据，配对失败 |
| **短距 S1_D2 未贴好** | SSR 校正失效，长距结果失真 |
| 漏光 / 饱和 | 光强异常，曲线乱跳 |

**皮肤检查**是在正式开测前，用 **3～5 秒** 短采集判断佩戴质量，并给出可操作的提示（类似 INVOS 的 **SSI 信号强度条**），而不是等采完才发现 `processed_output.csv` 生成失败。

---

## 2. 和皮肤检查、VOT、正式采集的区别

三者不要混为一谈：

| 功能 | 目的 | 时长 | 看几条曲线 | 当前是否已有 |
|------|------|------|------------|--------------|
| **皮肤检查** | 探头是否贴好、10 路光强是否正常 | 3～5 s | **10 路**（2 接收源 × 5 波长） | ❌ 无 |
| **VOT** | 缺血-再灌注激发试验，算 OS/RS 斜率 | 数分钟 | **1 路** rSO₂（如 `S1_D1_ssr`） | ❌ 无 |
| **正式采集** | 监护 / 实验 / 离线分析 | 用户定 | 在线 HbO/HbR/Cyt/rSO₂ | ✅ 已有 |

```text
建议产品流程：

  连接 PC → 皮肤检查（10 路光强）→ 通过 → 正式采集
                                    ↘ 可选：VOT（1 路 rSO₂ + 事件打点）
```

**和 VOT 的关键区别：**

- 皮肤检查看 **原始光强 ADC**，在 MBLL 之前，要快、要逐路反馈。  
- VOT 看 **rSO₂ 随时间变化**，要事件点（充气/放气），通常 **单通道** 即可。  
- 皮肤检查 **必须长距 + 短距都合格**；VOT 分析 **只用一条输出通道**（默认 `S1_D1_ssr`）。

---

## 3. 推荐方案总览

### 3.1 算法：三层组合（不用 SCI/SQI 做主判据）

| 层级 | 检查什么 | 通俗理解 |
|------|----------|----------|
| **第 1 层** | 光强是否在合理范围 | 有没有「亮到能测、又没亮过头」 |
| **第 2 层** | 3～5 秒内光强是否稳定（CV） | 贴紧了应较稳，松了会乱跳 |
| **第 3 层** | 10 路是否都有数据 | 2 接收源 × 5 波长是否齐全 |

汇总输出：

- 每路通道：`pass` / `warn` / `fail` / `missing`  
- 每个接收源：是否通过（**S1_D1 与 S1_D2 都必须过**）  
- 总体：**SSI 1～5 格** + `ok: true/false`  

### 3.2 分工：PC 算、安卓展示（推荐）

```text
安卓：启流 3～5 s → 发 serial_data → 显示结果、引导调整
PC：  收原始帧 → 算 10 路光强/CV → 回传 skin_check_result
```

**不建议**安卓本地再实现一套光强判定（易与 PC 离线标准不一致）。  
**不需要**单独进程或新端口，仍在 `online_android` 包内加模块，共用 TCP 9000。

### 3.3 为什么不用 SCI / SQI？

| 方法 | 原理 | 不适合做默认皮肤检查的原因 |
|------|------|---------------------------|
| SCI / SQI | 看信号里 0.5～2.5 Hz 心跳成分 | 需 10～30 s；波长时分复用下单通道采样率不够 |
| 光强 + CV | 看原始 ADC 范围与短时稳定性 | 3～5 s 即可；与现有 `threshold_filter` 思路一致 |

SCI/SQI 可作为 **「深度检查 15～30 s」** 的可选二期功能，不作为默认预检阻塞条件。

---

## 4. 架构：要不要单独做模块

**要专门模块，但不要单独服务。**

```text
online_android/
├── skin_check.py            # 纯算法：evaluate_skin_check()
├── skin_check_session.py    # 预检状态机：定时评估、发结果
├── skin_check_reporter.py   # 可选：发 skin_check_result（或并入 reporter.py）
├── tcp_bridge.py            # 只加分发，不写算法
├── buffer.py                  # 复用 OnlineSampleBuffer
└── session.py / capture.py    # 编排：先预检 → 再正式采集
```

| 要做 | 不要做 |
|------|--------|
| `online_android` 内 2～3 个新文件 | 单独 `skin_check_server.py` |
| 预检阶段不启 `OnlineAnalysisWorker` | 预检跑 MBLL / 回传 `live_analysis_batch` |
| 预检不写正式 `all_groups.csv` | 把逻辑全塞进 `tcp_bridge.py` |

与现有分工对齐：`rso2.py`（算法）→ `worker.py`（调度）→ `reporter.py`（发送）。

---

## 5. 用户流程

```text
┌──────────┐                      ┌──────────┐                      ┌──────────┐
│ 安卓 App │                      │ PC 解码端 │                      │  下位机   │
└────┬─────┘                      └────┬─────┘                      └────┬─────┘
     │  TCP 连接 :9000                 │                                 │
     │ ───── hello ──────────────────> │                                 │
     │ <──── hello_ack ─────────────── │                                 │
     │                                 │                                 │
     │ 【皮肤检查】                     │                                 │
     │ ─ skin_check_start ───────────> │ 进入预检模式                     │
     │ <── skin_check_start_ack ────── │                                 │
     │ UART 启流 3～5 s                │                                 │
     │ ───── serial_data ────────────> │ 缓冲原始帧，不算 MBLL            │
     │ <── skin_check_result ───────── │ 每秒推送（10 路状态 + SSI）       │
     │ UI：红/黄/绿，引导调整           │                                 │
     │                                 │                                 │
     │ ok=true → 点「开始正式采集」     │                                 │
     │ ─ skin_check_done(proceed) ───> │ 退出预检，启在线分析             │
     │                                 │                                 │
     │ 【正式采集】（现有流程）          │                                 │
     │ ───── serial_data ────────────> │ live_analysis_batch …           │
     │ ───── bye ────────────────────> │ analysis_result                 │
```

**要点：**

- 皮肤检查与正式采集可 **同一次 TCP 连接** 完成。  
- 预检失败可重复「启流 → 检查」，无需重连。  
- 通过皮肤检查 **不等于** 能做 VOT；VOT 另立流程（见 FAQ）。

---

## 6. 检测算法（三层组合）

### 6.1 输入

预检窗口内（默认最近 **5 s**）的原始采样，格式同 `all_groups.csv`：

```text
Time (s), DetectorId, Channel, Wavelength, Value
```

按 `(DetectorId, Wavelength)` 分为 **10 组**：

| 接收源 | 波长（nm） |
|--------|------------|
| S1_D1（长距 3.0 cm） | 850 / 810 / 770 / 730 / 700 |
| S1_D2（短距 1.0 cm） | 850 / 810 / 770 / 730 / 700 |

`Wavelength = 0x00`（OFF）**不参与**计算（与主链路一致）。

列名示例：`S1_D1_850`、`S1_D2_700`（与 `config.intensity_column_for` 一致）。

---

### 6.2 第 1 层：光强门限

每通道算窗口内光强 **中位数** `median(I)`（抗尖峰优于均值）。

| 判定 | 条件 |
|------|------|
| 过低 | `median < LOW` → 脱落、遮挡、头发 |
| 正常 | `LOW ≤ median ≤ HIGH` |
| 过高 | `median > HIGH` → 饱和、漏光 |

**建议初值**（与 `fnirs_pipeline/preprocessing.py` 中 `threshold_filter` 对齐）：

| 配置项 | 默认值 |
|--------|--------|
| `SKIN_CHECK_INTENSITY_LOW` | `50000` |
| `SKIN_CHECK_INTENSITY_HIGH` | `300000` |

二期可为长距/短距、各波长设独立阈值（见 [§11 阈值标定](#11-阈值标定)）。

---

### 6.3 第 2 层：稳定性 CV

要求用户 **静止 3～5 秒**。每通道：

```text
CV = std(I) / mean(I)
```

| CV | 等级 |
|----|------|
| < 0.02 | 优 |
| 0.02 ~ 0.05 | 可接受 |
| > 0.05 | 差（松动、滑动） |

| 配置项 | 默认值 |
|--------|--------|
| `SKIN_CHECK_CV_GOOD` | `0.02` |
| `SKIN_CHECK_CV_WARN` | `0.05` |
| `SKIN_CHECK_MIN_SAMPLES` | `5`（少于此不算 CV，判 `insufficient_data`） |

---

### 6.4 第 3 层：通道完整性（硬条件）

- 10 路在窗口内均有样本  
- 每路样本数 ≥ `SKIN_CHECK_MIN_SAMPLES`  
- **S1_D1 五波长全过 且 S1_D2 五波长全过**（短距关系 SSR，不能只验长距）

---

### 6.5 单通道 status 合并

| status | 条件 |
|--------|------|
| `pass` | 光强正常 + CV 优或可接受 + 样本够 |
| `warn` | 光强正常但 CV 偏差，或光强在边界附近 |
| `fail` | 光强超限，或 CV 太差 |
| `missing` | 无数据 |
| `insufficient_data` | 有数据但点数 < `MIN_SAMPLES` |

`reason` 字段供安卓显示：`intensity_low`、`intensity_high`、`cv_high`、`no_samples`。

---

### 6.6 SSI 总分（1～5 格）

| SSI | 建议条件 |
|-----|----------|
| **5** | 10/10 路 `pass`，CV 均 < 0.02，双接收源均过 |
| **4** | 10/10 路 `pass`，CV 可接受 |
| **3** | 8～9 路 pass，或 1 路 warn |
| **2** | 6～7 路 pass，或短距未全过 |
| **1** | 多路 fail / missing |

**总体 `ok: true` 建议：**

```text
SSI >= 4（可配置 SKIN_CHECK_MIN_SSI_PASS）
且 S1_D1 五波长全部 pass
且 S1_D2 五波长全部 pass
```

---

### 6.7 核心代码接口（建议）

```python
# online_android/skin_check.py

@dataclass(frozen=True)
class SkinCheckSettings:
    window_s: float = 5.0
    intensity_low: float = 50_000
    intensity_high: float = 300_000
    cv_good: float = 0.02
    cv_warn: float = 0.05
    min_samples: int = 5
    min_ssi_pass: int = 4

def evaluate_skin_check(
    raw_df: pd.DataFrame,
    settings: SkinCheckSettings | None = None,
) -> dict[str, Any]:
    """返回 skin_check_result 的 body，可直接 JSON 序列化。"""
```

**不调用：** `prepare_interleaved_dataframe`、`process_csv_dataset`、MBLL、SSR。

---

## 7. TCP 协议扩展

在 `online_android/安卓上位机tcp通讯协议.md` 中 **新增** 下列类型（`ver=1`，成帧方式不变：4 字节大端长度 + UTF-8 JSON）。

### 7.1 `skin_check_start`（安卓 → PC）

进入皮肤检查；接下来几秒为预检，非正式实验。

```json
{
  "ver": 1,
  "type": "skin_check_start",
  "seq": 10,
  "ts_ms": 1717234568000,
  "body": {
    "window_s": 5.0,
    "require_both_detectors": true
  }
}
```

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `window_s` | number | 否 | 评估窗口，默认 `5.0` |
| `require_both_detectors` | bool | 否 | 是否要求 S1_D1+S1_D2 都过，默认 `true` |

---

### 7.2 `skin_check_start_ack`（PC → 安卓）

```json
{
  "ver": 1,
  "type": "skin_check_start_ack",
  "seq": 5,
  "body": {
    "ref_seq": 10,
    "ok": true,
    "window_s": 5.0,
    "message": ""
  }
}
```

---

### 7.3 `skin_check_result`（PC → 安卓）

**推荐：** PC 在预检期间每 **1 s** 主动推送（安卓无需反复 `request`）。

```json
{
  "ver": 1,
  "type": "skin_check_result",
  "seq": 6,
  "ts_ms": 1717234568500,
  "body": {
    "ok": false,
    "ssi": 3,
    "window_s": 5.0,
    "sample_count_total": 1280,
    "message": "",
    "detectors": [
      {"name": "S1_D1", "pass": true, "pass_count": 5, "total": 5},
      {"name": "S1_D2", "pass": false, "pass_count": 3, "total": 5}
    ],
    "channels": [
      {
        "name": "S1_D2_850",
        "detector_id": 2,
        "wavelength_code": 1,
        "intensity_median": 42000,
        "cv": 0.062,
        "sample_count": 58,
        "status": "fail",
        "reason": "intensity_low"
      }
    ],
    "thresholds": {
      "intensity_low": 50000,
      "intensity_high": 300000,
      "cv_good": 0.02,
      "cv_warn": 0.05
    }
  }
}
```

| 字段 | 说明 |
|------|------|
| `ok` | 是否允许进入正式采集 |
| `ssi` | 1～5，仿 INVOS 信号强度条 |
| `channels[].status` | `pass` / `warn` / `fail` / `missing` / `insufficient_data` |
| `channels[].reason` | 失败原因码，安卓映射中文文案 |
| `thresholds` | 当前阈值，便于调试页展示 |

---

### 7.4 `skin_check_done`（安卓 → PC）

```json
{
  "ver": 1,
  "type": "skin_check_done",
  "seq": 20,
  "body": {
    "action": "proceed"
  }
}
```

| `action` | 含义 |
|----------|------|
| `proceed` | 预检通过，进入正式采集 |
| `cancel` | 放弃本次会话 |
| `retry` | 重新预检（可选） |

PC 收到 `proceed` 后：退出预检模式，启动 `create_online_session`，打开 `all_groups.csv` 写入。

---

### 7.5 各阶段允许的消息

| 阶段 | 安卓 → PC | PC → 安卓 | 禁止 |
|------|-----------|-----------|------|
| 皮肤检查 | `hello`, `skin_check_start`, `serial_data`, `skin_check_done` | `hello_ack`, `skin_check_start_ack`, `skin_check_result` | `live_analysis_batch`, `analysis_result` |
| 正式采集 | `serial_data`, `set_baseline`, `bye` | `live_analysis_batch`, `set_baseline_ack`, `bye_ack`, `analysis_result` | `skin_check_result` |

未知 `type` 仍回 `UNSUPPORTED_TYPE`（与现有一致）。

---

## 8. PC 端改造清单

### 8.1 新建 `online_android/skin_check.py`

- [ ] `SkinCheckSettings` 数据类  
- [ ] `evaluate_skin_check(raw_df) -> dict`  
- [ ] 过滤 `WAVELENGTH_OFF_CODE`  
- [ ] 按 `DETECTOR_CHANNELS` × `WAVELENGTH_CHANNELS` 生成 10 路结果  
- [ ] 算 median、CV、status、SSI、`ok`  
- [ ] 单元测试用构造 `DataFrame`（见 [§10](#10-测试清单)）  

### 8.2 新建 `online_android/skin_check_session.py`

- [ ] 状态：`idle` → `skin_checking` → `ready_for_capture`  
- [ ] 复用 `OnlineSampleBuffer`，只 `append` 不写 MBLL  
- [ ] 定时器每 `SKIN_CHECK_UPDATE_INTERVAL_S`（默认 1 s）调用 `evaluate_skin_check`  
- [ ] 调用 `send_skin_check_result`  
- [ ] 收到 `skin_check_done(proceed)` 后停止定时器，交棒正式 `OnlineCaptureSession`  

### 8.3 修改 `online_android/tcp_bridge.py`

- [ ] `_handle_message` 增加：`skin_check_start`、`skin_check_done`  
- [ ] `send_skin_check_start_ack`、`send_skin_check_result`  
- [ ] 预检模式下：`serial_data` 只写 buffer，**不** 喂 `OnlineAnalysisWorker`  
- [ ] 注册 `skin_check` 回调（模式同 `set_baseline_handler`）  

### 8.4 修改 `fnirs_pipeline/capture.py`

- [ ] 连接后等待/处理 `skin_check_start`（或 CLI `--skin_check` 强制预检）  
- [ ] 预检阶段 **不** 打开正式 `all_groups.csv`（可选写 `skin_check_debug.csv`）  
- [ ] `skin_check_done(proceed)` 后再 `create_online_session` 并写 CSV  

### 8.5 修改 `config.py`

```python
# ---------------------------------------------------------------------------
# 皮肤检查（佩戴质量预检）
# ---------------------------------------------------------------------------
SKIN_CHECK_ENABLED = True
SKIN_CHECK_WINDOW_S = 5.0
SKIN_CHECK_UPDATE_INTERVAL_S = 1.0
SKIN_CHECK_INTENSITY_LOW = 50_000
SKIN_CHECK_INTENSITY_HIGH = 300_000
SKIN_CHECK_CV_GOOD = 0.02
SKIN_CHECK_CV_WARN = 0.05
SKIN_CHECK_MIN_SAMPLES = 5
SKIN_CHECK_MIN_SSI_PASS = 4
```

- [ ] 二期：`SKIN_CHECK_INTENSITY_LOW_S1_D2` 等分通道阈值  

### 8.6 其它

- [ ] `online_android/__init__.py` 导出 `evaluate_skin_check`（按需）  
- [ ] `fNIRS_processing.py` 可选：`--skin_check` / `--no_skin_check`  
- [ ] 更新 `安卓上位机tcp通讯协议.md` 第 5.10～5.13 节与时序图  
- [ ] `README.md` / `fNIRS_processing_pipeline.md` 加预检分支说明（可选）  

### 8.7 与现有代码的关系

| 现有代码 | 关系 |
|----------|------|
| `preprocessing.threshold_filter` | 光强上下限 **参考同一量级**，预检不调用该函数改数据 |
| `adc_live.py` | 本地串口看光强，**仅调试**；产品路径走安卓 TCP |
| `OnlineAnalysisWorker` | 预检阶段 **不得启动** |
| `live_analysis_batch` | 预检阶段 **不得发送** |

---

## 9. 安卓端改造清单

### 9.1 UI

- [ ] 「皮肤检查」入口（正式采集前）  
- [ ] **SSI 1～5 格**（可仿 INVOS 绿条）  
- [ ] **两块区域**：S1_D1 / S1_D2，各 5 个波长圆点（绿/黄/红）  
- [ ] `ok=false` 时根据 `reason` 显示文案，例如：  

| reason | 建议文案 |
|--------|----------|
| `intensity_low` | 光强过低，请压紧探头 |
| `intensity_high` | 光强过高，请检查漏光或贴合 |
| `cv_high` | 信号不稳定，请保持静止并调整贴合 |
| `no_samples` | 该通道无数据，请拨开头发 |
| `insufficient_data` | 采样不足，请延长预检或检查连接 |

- [ ] `ok=true` 后启用「开始正式采集」  
- [ ] 短距 S1_D2 失败时 **单独高亮**（用户常只关注长距）  
- [ ] 提示：「请贴紧额头，保持静止 5 秒」  

### 9.2 协议

- [ ] 连接后发 `hello`，收 `hello_ack`  
- [ ] 点「皮肤检查」→ UART 启流 → 发 `skin_check_start`  
- [ ] 持续 `serial_data`  
- [ ] 收 `skin_check_result`，刷新 UI  
- [ ] 通过后发 `skin_check_done(action=proceed)`  
- [ ] 正式采集流程不变  

### 9.3 可选

- [ ] 调试模式「跳过皮肤检查」（默认关闭）  
- [ ] 展示 `thresholds` 供现场标定  

---

## 10. 测试清单

### 10.1 单元测试（`skin_check.py`）

| 用例 | 期望 |
|------|------|
| 10 路光强 ~17 万，CV 0.01 | `ok=true`, `ssi=5` |
| S1_D2_850 均值 1 万 | 该路 `fail`, `ok=false` |
| 某路 CV 0.08 | `fail` 或 `warn` |
| 仅 S1_D1 五路有数据 | S1_D2 `missing`, `ok=false` |
| 每路仅 2 点 | `insufficient_data` |
| 含 Wavelength=0x00 | 不参与计算 |

### 10.2 联调（安卓 + PC）

- [ ] `skin_check_start` → `skin_check_start_ack`  
- [ ] 5 s 内 ≥1 条 `skin_check_result`  
- [ ] 松动探头 → SSI 降、`ok=false`  
- [ ] 贴紧静止 → `ok=true`  
- [ ] 预检通过后正式采集，`live_analysis_batch` 正常  
- [ ] 预检阶段 **无** `live_analysis_batch`  

### 10.3 回归

- [ ] 不发 `skin_check_start` 时行为与改造前 **完全一致**  
- [ ] `set_baseline`、`bye`、`analysis_result` 不受影响  

---

## 11. 阈值标定

1. 从 `result_table/*/all_groups.csv` 选 **10 次好采集、10 次差采集**（人工标注）。  
2. 对每次预检窗口算 10 路 median、CV，画散点图。  
3. 调整 `INTENSITY_LOW/HIGH`，使好采集多 pass、差采集多 fail。  
4. **长距 S1_D1 与短距 S1_D2 分开标定**（二期）。  
5. 各波长可独立阈值（850 nm 与 700 nm 动态范围不同）。  
6. 定稿写入 `config.py`，`skin_check_result.thresholds` 回传便于安卓对齐。  

本地调试可参考 `adc_live.py` 阈值控件（默认 50000～300000），但产品路径以 TCP 预检为准。

---

## 12. 分阶段计划

### 第一阶段（最小可用，约 1～2 周）

| # | 任务 | 产出 |
|---|------|------|
| 1 | `skin_check.py` | 可单测 |
| 2 | `skin_check_session.py` + `tcp_bridge` 协议 | PC 可 mock 联调 |
| 3 | `capture.py` 预检编排 | 端到端预检 |
| 4 | 安卓 UI + 协议 | 真机演示 |
| 5 | 更新协议 md | 文档同步 |

### 第二阶段

- [ ] 长短距 / 分波长阈值  
- [ ] `skin_check_debug.csv` 落盘  
- [ ] PC `--live_plot` 预检光强条  

### 第三阶段（可选）

- [ ] 延长预检 15～30 s + 简化 SCI  
- [ ] 采集中 SSI 掉格报警（贴合监测）  

---

## 13. FAQ

**Q：皮肤检查和 `threshold_filter` 什么关系？**  
A：思路相同（光强是否合理）。`threshold_filter` 用于离线清洗数据；皮肤检查用于 **采集前** 判断，且增加 CV 与 10 路完整性，**不修改**原始数据。

**Q：预检数据写入 `all_groups.csv` 吗？**  
A：**不建议**写入正式文件。需要排查时可写 `skin_check_debug.csv`。

**Q：皮肤检查和 VOT 能合并吗？**  
A：**不要合并**。皮肤检查 = 10 路光强、3～5 s、贴没贴好；VOT = 1 路 rSO₂、事件点、算 OS/RS。流程上建议：皮肤检查 →（可选）VOT → 正式采集。

**Q：VOT 只要一个通道，皮肤检查呢？**  
A：皮肤检查要 **10 路都过**（含短距 S1_D2），因为 SSR 依赖短距；VOT 分析只用 `OUTPUT_CHANNEL` 一条 rSO₂。

**Q：安卓能自己做皮肤检查吗？**  
A：可以解析 `serial_data` 本地算，但与 PC 标准易分叉。**推荐 PC 算、安卓展示**。

**Q：和 INVOS SSI 一样吗？**  
A：形态类似（1～5 格），但你们是 **10 路近红外光强 + CV**，不是 INVOS 的单一 rSO₂ 信号条；SSI 映射规则需自行标定。

**Q：没有 PC 能预检吗？**  
A：安卓可对原始帧本地算光强门限，但当前产品架构是 **PC 解码端**，无 PC 则只能安卓简化版或不做预检。

---

## 14. 相关文件索引

| 文件 | 作用 |
|------|------|
| `fNIRS_processing.py` | 主入口 |
| `config.py` | 几何、波长、阈值（待扩展皮肤检查块） |
| `fnirs_pipeline/capture.py` | TCP 采集编排 |
| `fnirs_pipeline/preprocessing.py` | `threshold_filter` 参考 |
| `online_android/buffer.py` | 原始采样缓冲 |
| `online_android/tcp_bridge.py` | TCP 协议 |
| `online_android/session.py` | 正式在线分析会话 |
| `online_android/安卓上位机tcp通讯协议.md` | 协议文档（待更新） |
| `adc_live.py` | 本地串口看光强（调试，非产品路径） |

---

## 修订记录

| 日期 | 版本 | 说明 |
|------|------|------|
| 2026-07-06 | v1 | 初稿：三层方案 + PC/安卓清单 + 协议草案 |
| 2026-07-06 | v2 | 重写：明确与 VOT/正式采集边界；PC 算安卓展示；10 路 vs 单通道；模块架构；完整协议与 FAQ |
