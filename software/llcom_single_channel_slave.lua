-- LLCOM single-channel fNIRS slave simulator
--
-- Rewritten for the same LLCOM runtime style as the user's working example:
--   - receive callback: uartReceive(data)
--   - send API       : apiSendUartData(binary_string)
--   - timer/task API : sys.taskInit / sys.wait
--   - logging API    : log.info / log.warn

------------------------------------------------
-- 配置区
------------------------------------------------

-- 26B 协议固定头
local FRAME_H1 = 0x55
local FRAME_H2 = 0xAA
local FRAME_LEN = 0x1A

-- 帧类型
local TYPE_CMD  = 0x01
local TYPE_DATA = 0x02
local TYPE_ACK  = 0x03

-- 波长编码和当前单传感器编号
local WL_660 = 0x00
local WL_940 = 0x01
local SENSOR_ID = 0x00

-- 每 WAKE_MS 毫秒醒来一次，连续发 BURST_FRAMES 帧。目标约 1000 Hz。
-- 若平台把 sys.wait 放大（如 wait(10)≈1s），则加大 BURST 用较少唤醒凑够帧数。
local WAKE_MS = 10
local BURST_FRAMES = 100

-- 每个波长阶段发送的点数（660 发满 5 个再切到 940，反之亦然）
local POINTS_PER_WAVELENGTH = 5

-- 采样值随机范围 [VALUE_MIN, VALUE_MAX]，单位 ADC 计数，避免超出 0..4095
local VALUE_MIN = 800
local VALUE_MAX = 3200

------------------------------------------------
-- 运行状态
------------------------------------------------

local streaming = false
local intensity_ma = 300
local wavelength_code = WL_660
local points_in_phase = 0   -- 当前波长阶段已发送的点数，满 POINTS_PER_WAVELENGTH 后切换波长
local recv_buf = ""

------------------------------------------------
-- 工具函数
------------------------------------------------

local function log_info(tag, ...)
    if log and log.info then
        log.info(tag, ...)
    elseif print then
        local parts = {}
        for i = 1, select("#", ...) do
            parts[#parts + 1] = tostring(select(i, ...))
        end
        print("[llcom-sim][" .. tostring(tag) .. "] " .. table.concat(parts, " "))
    end
end

local function log_warn(tag, ...)
    if log and log.warn then
        log.warn(tag, ...)
    elseif print then
        local parts = {}
        for i = 1, select("#", ...) do
            parts[#parts + 1] = tostring(select(i, ...))
        end
        print("[llcom-sim][WARN][" .. tostring(tag) .. "] " .. table.concat(parts, " "))
    end
end

local function bytes_to_bin(bytes)
    -- 把 Lua 数值数组转成二进制串，供 apiSendUartData 发送。
    return string.char(table.unpack(bytes))
end

local function bytes_to_hex(bytes)
    local out = {}
    for i = 1, #bytes do
        out[#out + 1] = string.format("%02X", bytes[i])
    end
    return table.concat(out, " ")
end

local function hex_to_bin_string(hex_text)
    if type(hex_text) ~= "string" then
        return nil
    end

    if hex_text.fromHex then
        return hex_text:fromHex()
    end

    local cleaned = hex_text:gsub("0[xX]", "")
    cleaned = cleaned:gsub("[^%x]", "")
    if #cleaned < 2 or (#cleaned % 2) ~= 0 then
        return nil
    end

    local bytes = {}
    for i = 1, #cleaned, 2 do
        bytes[#bytes + 1] = tonumber(cleaned:sub(i, i + 1), 16)
    end
    return bytes_to_bin(bytes)
end

local function looks_like_hex_text(data)
    if type(data) ~= "string" or data == "" then
        return false
    end
    if data:find("[^%x%s,;:xX%-_]") then
        return false
    end
    local hex_chars = data:gsub("[^%x]", "")
    return #hex_chars >= 4 and (#hex_chars % 2) == 0
end

local function normalize_rx_data(data)
    -- llcom 有时会把手工输入的 Hex 文本传给脚本，这里自动转回原始字节流。
    if looks_like_hex_text(data) then
        local raw = hex_to_bin_string(data)
        if raw then
            log_info("uart_rx", "hex-text input detected and converted")
            return raw
        end
    end
    return data
end

local function calc_checksum(length_byte, frame_type, payload_bytes)
    -- 与上位机一致：长度字节 + 帧类型 + payload 的累加和低字节。
    local sum = length_byte + frame_type
    for i = 1, #payload_bytes do
        sum = sum + payload_bytes[i]
    end
    return sum % 0x100
end

local function build_frame(frame_type, payload_bytes)
    -- 统一构造 26B 帧，未使用的 payload 位全部补 0。
    local payload = {}
    for i = 1, 21 do
        payload[i] = payload_bytes[i] or 0x00
    end

    local bytes = { FRAME_H1, FRAME_H2, FRAME_LEN, frame_type }
    for i = 1, 21 do
        bytes[#bytes + 1] = payload[i]
    end
    bytes[#bytes + 1] = calc_checksum(FRAME_LEN, frame_type, payload)
    return bytes
end

local function send_frame(frame_type, payload_bytes)
    -- 真正调用 llcom 运行时的 apiSendUartData 把二进制串发出去。
    local frame = build_frame(frame_type, payload_bytes)
    local frame_bin = bytes_to_bin(frame)
    local ok, err = pcall(apiSendUartData, frame_bin)
    if not ok then
        log_warn("uart_tx", "apiSendUartData failed:", err)
        return false
    end
    log_info("uart_tx", bytes_to_hex(frame))
    return true
end

local function send_ack()
    -- ACK 帧 payload 当前全 0。
    send_frame(TYPE_ACK, {})
end

local function next_sample_value()
    -- 在 [VALUE_MIN, VALUE_MAX] 内随机生成 ADC 值，非正余弦。
    local value = math.random(VALUE_MIN, VALUE_MAX)
    if value < 0 then value = 0 end
    if value > 4095 then value = 4095 end
    return value
end

local function send_data_frame()
    -- 当前波长下发一帧数据；本阶段发满 POINTS_PER_WAVELENGTH 个点后切换到另一波长。
    local value = next_sample_value()
    local low = value % 0x100
    local high = math.floor(value / 0x100)
    local payload = {
        wavelength_code,
        SENSOR_ID,
        low,
        high,
    }
    payload[21] = 0x00
    send_frame(TYPE_DATA, payload)

    points_in_phase = points_in_phase + 1
    if points_in_phase >= POINTS_PER_WAVELENGTH then
        points_in_phase = 0
        wavelength_code = (wavelength_code == WL_660) and WL_940 or WL_660
    end
end

------------------------------------------------
-- 断帧与协议处理
------------------------------------------------

local function parse_one_frame(buf)
    -- 从接收缓冲区里尽量解析出一帧完整 26B 协议帧。
    local total = 26
    if #buf < total then
        return nil, buf
    end

    local h1, h2 = buf:byte(1), buf:byte(2)
    if h1 ~= FRAME_H1 or h2 ~= FRAME_H2 then
        return nil, buf:sub(2)
    end

    local frame_len = buf:byte(3)
    if frame_len ~= FRAME_LEN then
        log_warn("frame", string.format("invalid len 0x%02X", frame_len))
        return nil, buf:sub(2)
    end

    if #buf < frame_len then
        return nil, buf
    end

    local frame = buf:sub(1, frame_len)
    local frame_type = frame:byte(4)
    local payload = { frame:byte(5, 25) }
    local checksum = frame:byte(26)
    local expected = calc_checksum(frame_len, frame_type, payload)
    if checksum ~= expected then
        log_warn("frame", string.format("checksum mismatch got=0x%02X expected=0x%02X", checksum, expected))
        return nil, buf:sub(2)
    end

    return {
        frame_type = frame_type,
        payload = payload,
        raw = frame,
    }, buf:sub(frame_len + 1)
end

local function handle_command(payload)
    -- 命令帧当前只用到：启停 + 光强。
    local stream_enable = payload[1] or 0x00
    local intensity_high = payload[2] or 0x00
    local intensity_low = payload[3] or 0x00

    -- 先更新状态，再回 ACK；停止时也会先停流再回 ACK。
    streaming = (stream_enable ~= 0x00)
    intensity_ma = intensity_high * 0x100 + intensity_low
    if intensity_ma <= 0 then
        intensity_ma = 300
    end

    log_info("frame", "command frame received", "streaming=", tostring(streaming), "intensity=", intensity_ma)
    send_ack()
end

local function handle_frame(frame)
    -- 目前只处理命令帧；ACK 仅打印日志。
    if frame.frame_type == TYPE_CMD then
        handle_command(frame.payload)
    elseif frame.frame_type == TYPE_ACK then
        log_info("frame", "host ACK received")
    else
        log_info("frame", string.format("frame type 0x%02X received", frame.frame_type))
    end
end

local function drain_frames()
    -- 持续从 recv_buf 中捞完整帧，直到不够一帧为止。
    while #recv_buf >= 3 do
        local b1, b2 = recv_buf:byte(1), recv_buf:byte(2)
        if b1 ~= FRAME_H1 or b2 ~= FRAME_H2 then
            recv_buf = recv_buf:sub(2)
            goto continue
        end

        local frame_len = recv_buf:byte(3)
        if frame_len ~= FRAME_LEN then
            recv_buf = recv_buf:sub(2)
            goto continue
        end

        if #recv_buf < frame_len then
            break
        end

        local frame
        frame, recv_buf = parse_one_frame(recv_buf)
        if frame then
            if frame.raw.toHex then
                log_info("uart_rx", frame.raw:toHex(" "))
            else
                log_info("uart_rx", bytes_to_hex({ frame.raw:byte(1, #frame.raw) }))
            end
            handle_frame(frame)
        end

        ::continue::
    end
end

------------------------------------------------
-- LLCOM 回调
------------------------------------------------

uartReceive = function(data)
    -- llcom 的串口接收入口：这里只做缓存和断帧。
    if not data or #data == 0 then
        return
    end

    log_info("uart_rx", "len=", #data)
    recv_buf = recv_buf .. normalize_rx_data(data)
    drain_frames()
end

------------------------------------------------
-- 周期发送任务
------------------------------------------------

sys.taskInit(function()
    -- 每 WAKE_MS 毫秒发 BURST_FRAMES 帧，避免 sys.wait(1) 被圆整为 100 ms 导致只有 10 Hz。
    log_info("sys", "single-channel slave simulator initialized")
    while true do
        if streaming then
            for _ = 1, BURST_FRAMES do
                send_data_frame()
            end
        end
        sys.wait(WAKE_MS)
    end
end)
