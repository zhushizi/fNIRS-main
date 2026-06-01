# 安卓上位机 TCP 通讯协议

| 项目 | 说明 |
|------|------|
| 协议名称 | Host TCP Protocol（第一层 / 应用层） |
| 版本 | `1` |
| 字符编码 | UTF-8（JSON 文本） |
| 传输 | TCP |
| 关联文档 | 下位机串口 **26B 帧协议**（`protocol.py`、`config.py`，第二层） |

---

## 1. 协议定位

本协议用于 **安卓上位机** 与 **解码/处理端（PC 或其它主机）** 之间交换数据，不替代下位机串口协议。

```
┌────────────┐   本协议（JSON + 长度头）   ┌────────────┐   26B 二进制帧   ┌────────────┐
│ 解码端      │ ◄────────────────────────► │ 安卓上位机  │ ◄──────────────► │ 下位机      │
│ (fNIRS 等)  │         TCP                │ (串口唯一)  │      UART        │            │
└────────────┘                            └────────────┘                  └────────────┘
     第一层                                      桥接                           第二层
```

**原则**

1. **串口只由安卓打开**；解码端不直接访问 UART。
2. `payload` 中的二进制一律用 **Base64** 表示，禁止把串口原始字节当 UTF-8 字符串塞进 JSON。
3. 一条 TCP 应用消息 **≠** 一帧 26B；`serial_data` 可携带任意长度串口读块，解码端自行缓冲、找帧头 `55 AA`。
4. 命令推荐 **透传完整 26B 命令帧**（与上行对称），避免 PC 与安卓各维护一套组帧逻辑。

---

## 2. 连接角色

| 角色 | 建议 | 说明 |
|------|------|------|
| **TCP 服务端** | 解码端（PC） | 固定 IP/端口，实验室内 PC 监听 |
| **TCP 客户端** | 安卓上位机 | 主动 `connect`，断线可重连 |

默认参数（实现时可配置）：

| 参数 | 默认值 |
|------|--------|
| 监听地址 | `0.0.0.0` |
| 端口 | `9000` |
| 单连接 | 同时只服务 **1** 个客户端；新连接可踢掉旧连接或拒绝（实现时二选一并写进 release note） |

连接建立后，**建议** 安卓先发 `hello`，解码端回 `hello_ack`，再开始 `serial_data` / `serial_cmd`（见 §5.1）。

---

## 3. 应用层成帧（所有消息通用）

TCP 是字节流，必须用 **长度前缀** 界定一条应用消息。

### 3.1 帧格式

```
 0                   1                   2                   3
 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                        body_length (uint32 BE)               |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
|                     JSON body (body_length 字节)              |
|                              ...                              |
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
```

| 字段 | 长度 | 说明 |
|------|------|------|
| `body_length` | 4 字节 | 大端无符号整数，表示后续 JSON 字节数 |
| JSON body | `body_length` | UTF-8 编码的 JSON 对象 |

**约束**

- `body_length` 范围：`1` ~ `4_194_304`（4 MiB，可协商调小）。
- JSON 必须为 **单个对象**（一条消息一个根对象）。
- 不允许一次 `send` 里拼接两条消息；不允许 JSON 后拖垃圾字节。

### 3.2 收发伪代码

**发送**

```text
body = UTF8( JSON.stringify(message) )
send( uint32_be(len(body)) + body )
```

**接收**

```text
hdr = recv_exact(4)
n   = uint32_be(hdr)
body = recv_exact(n)
message = JSON.parse(UTF8(body))
```

---

## 4. JSON 公共字段

每条消息均包含：

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `ver` | int | 是 | 协议版本，当前固定 `1` |
| `type` | string | 是 | 消息类型，见 §5 |
| `seq` | int | 是 | 发送方单调递增序号，从 `1` 开始；重连用 **新 seq** |
| `ts_ms` | int | 否 | 发送方 Unix 时间戳（毫秒），便于对时与排错 |

类型相关字段放在 `body` 对象内（除 `error` 可提升为顶层，见 §5.8）。

**示例骨架**

```json
{
  "ver": 1,
  "type": "serial_data",
  "seq": 42,
  "ts_ms": 1717234567890,
  "body": { }
}
```

---

## 5. 消息类型

### 5.1 连接与握手

#### `hello`（安卓 → 解码端）

连接成功后 **建议首包**。

| `body` 字段 | 类型 | 必填 | 说明 |
|-------------|------|------|------|
| `client_id` | string | 否 | 设备/应用标识，如 `"fNIRS-Android-1.0"` |
| `serial_port` | string | 否 | 安卓侧串口名，仅日志用 |
| `baud_rate` | int | 否 | 默认 `115200` |

```json
{
  "ver": 1,
  "type": "hello",
  "seq": 1,
  "ts_ms": 1717234567890,
  "body": {
    "client_id": "fNIRS-Android-1.0",
    "baud_rate": 115200
  }
}
```

#### `hello_ack`（解码端 → 安卓）

| `body` 字段 | 类型 | 必填 | 说明 |
|-------------|------|------|------|
| `server_id` | string | 否 | 如 `"fNIRS-Decoder-1.0"` |
| `ok` | bool | 是 | 是否接受会话 |
| `reason` | string | 否 | `ok=false` 时说明原因 |

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

---

### 5.2 串口数据上行 `serial_data`（安卓 → 解码端）

将 **`InputStream.read()` 读到的原始字节块** 原样 Base64 后上报。块大小由安卓决定（如 64~4096 字节），**不要求对齐 26B**。

| `body` 字段 | 类型 | 必填 | 说明 |
|-------------|------|------|------|
| `payload_b64` | string | 是 | 串口原始字节的 Base64（标准 alphabet，带 padding） |
| `byte_len` | int | 否 | 解码后长度，用于校验 |

```json
{
  "ver": 1,
  "type": "serial_data",
  "seq": 100,
  "ts_ms": 1717234568000,
  "body": {
    "payload_b64": "VaqqGgACAAAAAAABAAAAZv8=",
    "byte_len": 52
  }
}
```

**解码端处理**

1. `base64.decode(payload_b64)` → `bytes`
2. 追加到接收缓冲区
3. 按第二层协议找 `55 AA`，解析 0x02 数据帧等

---

### 5.3 串口命令下行 `serial_cmd`（解码端 → 安卓）

将 **完整 26B 帧**（通常 0x01 命令帧）Base64 发给安卓，安卓 **原样写入串口**。

| `body` 字段 | 类型 | 必填 | 说明 |
|-------------|------|------|------|
| `payload_b64` | string | 是 | 一条或多条帧连续拼接的字节（见下） |
| `byte_len` | int | 否 | 解码后长度 |
| `expect_ack` | bool | 否 | 默认 `false`；若固件回 0x03 ACK，安卓可读串口并可选上报 |

**单条命令帧**（启流示例，payload 依 `build_command_frame`）：

```json
{
  "ver": 1,
  "type": "serial_cmd",
  "seq": 10,
  "ts_ms": 1717234567900,
  "body": {
    "payload_b64": "VaqqGgABAQAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=",
    "byte_len": 26,
    "expect_ack": false
  }
}
```

> 注：上式 `payload_b64` 为示意，实际须按真实 checksum 计算后填入。

**安卓动作**：`write(decoded_bytes)`；若 `expect_ack=true`，在超时内读串口，可将收到的字节再以 `serial_data` 形式回传，或仅用于本地判断成功。

#### `serial_cmd_ack`（安卓 → 解码端，可选）

| `body` 字段 | 类型 | 说明 |
|-------------|------|------|
| `ref_seq` | int | 对应 `serial_cmd.seq` |
| `ok` | bool | 是否认为发送成功 |
| `reason` | string | 失败原因 |
| `payload_b64` | string | 可选，串口读到的应答字节（含 0x03 ACK） |

---

### 5.4 语义化启停 `stream_control`（解码端 → 安卓，可选）

若希望安卓本地组 0x01 帧，可用本消息代替 `serial_cmd`（**二选一**，同一产品只用一种）。

| `body` 字段 | 类型 | 必填 | 说明 |
|-------------|------|------|------|
| `enabled` | bool | 是 | `true` 启流，`false` 停流 |
| `intensity_ma` | int | 否 | 0~255，默认 `200` |

```json
{
  "ver": 1,
  "type": "stream_control",
  "seq": 11,
  "body": {
    "enabled": true,
    "intensity_ma": 200
  }
}
```

**安卓动作**：按与 `protocol.build_command_frame` 相同规则组 26B 帧并写串口。

| `stream_control_ack` | 安卓 → 解码端 |
|----------------------|---------------|
| `body.ref_seq` | 对应请求的 `seq` |
| `body.ok` | 是否成功 |

---

### 5.5 心跳 `heartbeat`（双向）

空闲时建议 **每 5~30 秒** 一方发送，对方可回可不回。

```json
{
  "ver": 1,
  "type": "heartbeat",
  "seq": 200,
  "body": {}
}
```

收到后 **无需** 业务处理；可用于 NAT/防火墙保活。

---

### 5.6 会话结束 `bye`（双向）

```json
{
  "ver": 1,
  "type": "bye",
  "seq": 999,
  "body": {
    "reason": "user_stop"
  }
}
```

对端收到后应停止 `serial_data`、关闭串口流（若由该会话开启），并关闭 TCP。

---

### 5.7 错误 `error`（双向）

解析失败、Base64 非法、串口写失败等。

```json
{
  "ver": 1,
  "type": "error",
  "seq": 50,
  "body": {
    "ref_seq": 10,
    "code": "SERIAL_WRITE_FAILED",
    "message": "UsbSerial write returned 0 bytes"
  }
}
```

| `code` 建议值 | 含义 |
|---------------|------|
| `INVALID_JSON` | JSON 无法解析 |
| `INVALID_BASE64` | payload_b64 非法 |
| `LENGTH_MISMATCH` | byte_len 与解码长度不符 |
| `UNSUPPORTED_TYPE` | 未知 type |
| `SERIAL_OPEN_FAILED` | 安卓未打开串口 |
| `SERIAL_WRITE_FAILED` | 写串口失败 |
| `VERSION_MISMATCH` | ver 不支持 |

---

## 6. 与第二层（26B 串口协议）的关系

本协议 **不定义** 0x02 数据帧内部字段，仅透传字节。第二层要点（便于安卓/解码端对齐）：

| 项目 | 值 |
|------|-----|
| 帧头 | `55 AA` |
| 帧长 | 固定 26 字节 |
| 长度字节 | `0x1A` |
| 帧类型 | `0x01` 命令 / `0x02` 数据 / `0x03` ACK |
| 命令 payload[0] | `0x01` 启流，`0x00` 停流 |
| 命令 payload[1] | 光强（mA 量级，单字节 0~255） |
| 数据 payload[0] | 波长码：`0x00` OFF，`0x01` 940nm，`0x02` 660nm 等 |

解码端对 `serial_data` 解码后的字节流，处理方式与直连串口时 `FrameReader` 一致。

---

## 7. 典型业务流程

### 7.1 解码端发起采集（推荐：透传 `serial_cmd`）

```mermaid
sequenceDiagram
    participant PC as 解码端
    participant AND as 安卓上位机
    participant DEV as 下位机

    AND->>PC: TCP connect
    AND->>PC: hello
    PC->>AND: hello_ack (ok=true)
    PC->>AND: serial_cmd (0x01 启流帧 b64)
    AND->>DEV: UART write 26B
    loop 采集
        DEV->>AND: UART 0x02 数据
        AND->>PC: serial_data (read 块 b64)
        PC->>PC: 缓冲 + 26B 解帧 + 算法
    end
    PC->>AND: serial_cmd (0x01 停流帧 b64)
    AND->>DEV: UART write
    PC->>AND: bye
    AND->>PC: bye
```

### 7.2 仅安卓 UI 启停（解码端只收数）

- 安卓在本地 UI 发 0x01 / 停流，**不发** `serial_cmd`。
- 解码端只处理 `serial_data`；连接后仍可 `hello` / `hello_ack`。

### 7.3 使用 `stream_control` 启停

将 §7.1 中 `serial_cmd` 替换为 `stream_control` + `stream_control_ack`，安卓负责组 26B 帧。

---

## 8. 序号、重试与顺序

| 规则 | 说明 |
|------|------|
| `seq` | 每方独立递增；响应用 `ref_seq` 指向请求 |
| 顺序 | TCP 保证同连接字节序；`serial_data` 必须按发送顺序拼接缓冲 |
| 重试 | 命令失败可重发 **新 seq** 的 `serial_cmd`；禁止重复消费同一 `serial_data` |
| 幂等 | `stream_control(enabled=false)` 可多次发送 |

---

## 9. 实现检查清单

**安卓**

- [ ] TCP Client，`connect(PC_IP, 9000)`
- [ ] 串口 `read` → `serial_data.payload_b64`
- [ ] 收到 `serial_cmd` → Base64 解码 → `write`
- [ ] 可选：`stream_control` 本地组帧
- [ ] 断线重连 + `hello` 重发

**解码端**

- [ ] TCP Server 监听
- [ ] 长度前缀拆包 + JSON 解析
- [ ] `serial_data` → 缓冲 → 26B 解帧
- [ ] 启停：`serial_cmd` 或 `stream_control`
- [ ] 非法包回 `error`

---

## 10. 附录：Base64 与长度校验

```text
decoded = Base64Decode(payload_b64)
if byte_len is present and len(decoded) != byte_len:
    send error(LENGTH_MISMATCH)
```

- 使用标准 Base64（RFC 4648），不用 URL-safe 变体，除非双方显式约定。
- 空块：`payload_b64` 可省略或 `""`，解码长度为 0（一般不上报空 read）。

---

## 11. 版本演进

| ver | 变更 |
|-----|------|
| `1` | 首版：长度头 + JSON、`hello`、`serial_data`、`serial_cmd`、`stream_control`、心跳、`bye`、`error` |

不兼容变更时递增 `ver`；解码端对未知 `ver` 应 `hello_ack.ok=false` 或 `error(VERSION_MISMATCH)`。

---

## 12. 修订记录

| 日期 | 说明 |
|------|------|
| 2026-06-01 | 初稿 |
