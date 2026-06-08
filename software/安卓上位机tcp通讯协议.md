# 安卓上位机 TCP 通讯协议

| 项目 | 说明 |
|------|------|
| 协议名称 | Host TCP Protocol |
| 版本 | `1` |
| 字符编码 | UTF-8 JSON |
| 传输 | TCP |
| 关联协议 | 下位机 26B 串口帧协议（`protocol.py` / `config.py`） |

---

## 1. 协议定位

当前版本采用 **安卓主导启停、PC 被动收数并分析** 的流程。

```text
┌────────────┐   JSON + 4B长度头   ┌────────────┐   26B UART帧   ┌────────────┐
│ 解码端 PC   │ ◄────────────────► │ 安卓上位机  │ ◄────────────► │ 下位机      │
│ fNIRS      │                     │ 串口唯一方  │                │            │
└────────────┘                     └────────────┘                └────────────┘
```

原则：

1. 串口只由安卓打开，PC 不直接访问 UART。
2. 安卓本地负责启流/停流，PC 不下发串口启停命令。
3. 安卓把串口读到的原始字节块通过 `serial_data` 发给 PC。
4. PC 在接收 `serial_data` 过程中可周期性发送 `live_analysis_batch`，携带一批 HbO/HbR 曲线点用于实时绘图。
5. 安卓发送 `bye` 表示本次数据流结束；PC 回 `bye_ack` 后做最终汇总分析。
6. PC 最终分析完成后发送 `analysis_result`，携带本次 HbO/HbR 平均值用于测试显示。

---

## 2. 连接角色

| 角色 | 说明 |
|------|------|
| PC 解码端 | TCP Server，默认监听 `0.0.0.0:9000` |
| 安卓上位机 | TCP Client，主动连接 PC 的局域网 IP |

PC 启动：

```powershell
python software/fNIRS_processing.py -tcp_port
```

安卓连接：`PC_IP:9000`。

---

## 3. 应用层成帧

TCP 是字节流，每条 JSON 消息前必须加 4 字节大端长度头。

```text
+-------------------------------+
| body_length uint32 big-endian |
+-------------------------------+
| JSON body UTF-8 bytes         |
+-------------------------------+
```

发送伪代码：

```text
body = UTF8(JSON.stringify(message))
send(uint32_be(len(body)) + body)
```

约束：

- `body_length` 范围：`1` 到 `4_194_304`。
- JSON 根节点必须是对象。
- 一条应用消息只包含一个 JSON 对象。

---

## 4. JSON 公共字段

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `ver` | int | 是 | 协议版本，当前固定 `1` |
| `type` | string | 是 | 消息类型 |
| `seq` | int | 是 | 发送方单调递增序号，从 `1` 开始 |
| `ts_ms` | int | 否 | 发送方 Unix 时间戳，毫秒 |
| `body` | object | 是 | 类型相关字段 |

---

## 5. 消息类型

### 5.1 `hello`（安卓 -> PC）

安卓 TCP 连接成功后首包发送。

```json
{
  "ver": 1,
  "type": "hello",
  "seq": 1,
  "ts_ms": 1717234567890,
  "body": {
    "client_id": "fNIRS-Android-1.0",
    "serial_port": "USB0",
    "baud_rate": 115200
  }
}
```

### 5.2 `hello_ack`（PC -> 安卓）

PC 接受会话后回复。

```json
{
  "ver": 1,
  "type": "hello_ack",
  "seq": 1,
  "ts_ms": 1717234567891,
  "body": {
    "server_id": "fNIRS-Decoder-1.0",
    "ok": true
  }
}
```

### 5.3 `serial_data`（安卓 -> PC）

安卓将 `InputStream.read()` 读到的串口原始字节块原样 Base64 后上报。块大小由安卓决定，不要求对齐 26B。

| `body` 字段 | 类型 | 必填 | 说明 |
|-------------|------|------|------|
| `payload_b64` | string | 是 | 串口原始字节的标准 Base64 |
| `byte_len` | int | 否 | 解码后的字节数，用于校验 |

```json
{
  "ver": 1,
  "type": "serial_data",
  "seq": 100,
  "ts_ms": 1717234568000,
  "body": {
    "payload_b64": "VaqqGgACAAAAAAABAAAAZv8=",
    "byte_len": 26
  }
}
```

PC 处理方式：Base64 解码，追加到接收缓冲区，按第二层 26B 协议找帧头 `55 AA` 并解析 `0x02` 数据帧，写入本次 CSV。

### 5.4 `bye`（安卓 -> PC）

安卓在本次采集结束后发送。典型场景：用户在安卓 UI 点击停流后，安卓停止本次数据流，然后发送 `bye` 通知 PC 开始分析。

```json
{
  "ver": 1,
  "type": "bye",
  "seq": 999,
  "ts_ms": 1717234570000,
  "body": {
    "reason": "user_stop"
  }
}
```

### 5.5 `bye_ack`（PC -> 安卓）

PC 收到 `bye` 后立即回复，表示已经结束采集循环并即将开始分析。

```json
{
  "ver": 1,
  "type": "bye_ack",
  "seq": 2,
  "ts_ms": 1717234570001,
  "body": {
    "ref_seq": 999,
    "ok": true
  }
}
```

### 5.6 `live_analysis_batch`（PC -> 安卓）

PC 在采集过程中周期性发送在线分析结果。该消息用于安卓端实时曲线绘图；每批只包含上次回传后新增的点。

| `body` 字段 | 类型 | 必填 | 说明 |
|-------------|------|------|------|
| `ok` | bool | 是 | 是否成功生成本批在线结果 |
| `times` | number[] | 成功时是 | 曲线点时间，单位秒，相对本次采集开始 |
| `hbo` | number[] | 成功时是 | 与 `times` 对齐的 HbO 序列 |
| `hbr` | number[] | 成功时是 | 与 `times` 对齐的 HbR 序列 |
| `sample_count` | int | 是 | 本批点数 |
| `window_start_s` | number | 成功时是 | 本次滑动窗口起始时间 |
| `window_end_s` | number | 成功时是 | 本次滑动窗口结束时间 |
| `unit` | string | 成功时是 | 当前为 `a.u.` |
| `message` | string | 否 | 失败或调试说明 |

成功示例：

```json
{
  "ver": 1,
  "type": "live_analysis_batch",
  "seq": 3,
  "ts_ms": 1717234569000,
  "body": {
    "ok": true,
    "times": [30.125, 31.126, 32.127],
    "hbo": [0.0000012, 0.0000014, 0.0000015],
    "hbr": [-0.0000004, -0.0000005, -0.0000006],
    "sample_count": 3,
    "window_start_s": 2.125,
    "window_end_s": 32.127,
    "unit": "a.u."
  }
}
```

说明：

- 在线算法使用滑动窗口，前期数据不足时 PC 可能暂时不发送本消息。
- 安卓端可按 `times[i] / hbo[i] / hbr[i]` 直接追加到曲线。
- 最终准确结果仍以 `analysis_result` 和 `processed_output.csv` 为准。

### 5.7 `analysis_result`（PC -> 安卓）

PC 完成本次后处理后发送一条测试结果，包含 HbO/HbR 平均值。

成功示例：

```json
{
  "ver": 1,
  "type": "analysis_result",
  "seq": 3,
  "ts_ms": 1717234572000,
  "body": {
    "ok": true,
    "mean_hbo": 0.0000123,
    "mean_hbr": -0.0000045,
    "sample_count": 128,
    "unit": "a.u."
  }
}
```

失败示例：

```json
{
  "ver": 1,
  "type": "analysis_result",
  "seq": 3,
  "body": {
    "ok": false,
    "message": "No complete wavelength cycles were formed."
  }
}
```

### 5.8 `heartbeat`（双向，可选）

```json
{
  "ver": 1,
  "type": "heartbeat",
  "seq": 200,
  "body": {}
}
```

### 5.9 `error`（双向）

```json
{
  "ver": 1,
  "type": "error",
  "seq": 50,
  "body": {
    "ref_seq": 10,
    "code": "INVALID_BASE64",
    "message": "payload_b64 is not valid standard Base64"
  }
}
```

建议错误码：`INVALID_JSON`、`INVALID_BASE64`、`LENGTH_MISMATCH`、`UNSUPPORTED_TYPE`、`SERIAL_OPEN_FAILED`、`SERIAL_WRITE_FAILED`、`VERSION_MISMATCH`、`ANALYSIS_FAILED`。

---

## 6. 第二层 26B 串口协议关系

本 TCP 协议不解释下位机数据帧内部字段，只透传串口字节。PC 对 `serial_data` 解码后的字节流按现有 `FrameReader` 逻辑处理。

| 项目 | 值 |
|------|-----|
| 帧头 | `55 AA` |
| 帧长 | 固定 26 字节 |
| 长度字节 | `0x1A` |
| 帧类型 | `0x01` 命令 / `0x02` 数据 / `0x03` ACK |
| 数据 payload[0] | 波长码：`0x00` OFF，`0x01` 940nm，`0x02` 660nm 等 |

启流/停流命令帧由安卓本地生成并写入 UART，PC 不通过 TCP 下发。

---

## 7. 典型业务流程

```mermaid
sequenceDiagram
    participant PC as 解码端 PC
    participant AND as 安卓上位机
    participant DEV as 下位机

    PC->>PC: 监听 0.0.0.0:9000
    AND->>PC: TCP connect
    AND->>PC: hello
    PC->>AND: hello_ack(ok=true)
    AND->>DEV: UI 启流，UART 写 26B 启流命令
    loop 采集
        DEV->>AND: UART 0x02 数据帧
        AND->>PC: serial_data(payload_b64)
        PC->>PC: 缓冲 + 26B 解帧 + 写 CSV + 在线窗口分析
        PC-->>AND: live_analysis_batch(times, hbo, hbr)
    end
    AND->>DEV: UI 停流，UART 写 26B 停流命令
    AND->>PC: bye(reason=user_stop)
    PC->>AND: bye_ack(ok=true)
    PC->>PC: CSV 后处理，计算 HbO/HbR 平均值
    PC->>AND: analysis_result(mean_hbo, mean_hbr)
```

---

## 8. 实现检查清单

安卓：

- [ ] TCP Client 连接 `PC_IP:9000`
- [ ] 连接后发送 `hello`
- [ ] 收到 `hello_ack.ok=true` 后允许 UI 启流
- [ ] 串口 `read()` 到的原始字节用 `serial_data.payload_b64` 上报
- [ ] 采集过程中接收 `live_analysis_batch` 并追加绘图
- [ ] UI 停流后发送 `bye`
- [ ] 接收 `bye_ack`
- [ ] 接收并显示 `analysis_result`
- [ ] 串口或协议错误时发送 `error`

PC：

- [ ] TCP Server 监听
- [ ] 长度头拆包 + JSON 解析
- [ ] `hello` -> `hello_ack`
- [ ] `serial_data` -> 缓冲 -> 26B 解帧 -> CSV
- [ ] 采集过程中周期性发送 `live_analysis_batch`
- [ ] `bye` -> `bye_ack` -> 结束采集 -> 后处理
- [ ] 后处理完成 -> `analysis_result`
- [ ] 非法包回 `error`

---

## 9. 修订记录

| 日期 | 说明 |
|------|------|
| 2026-06-01 | 初稿 |
| 2026-06-02 | 精简为安卓主导启停：`hello` / `serial_data` / `bye` / `analysis_result` |
| 2026-06-03 | 增加在线曲线回传：`live_analysis_batch` |
