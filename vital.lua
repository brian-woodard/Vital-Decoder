
-- visual_protocol.lua
--
-- Wireshark Lua dissector for:
--   System Features (Block Code 0)
--   High Precision Expanded Ownship (Block Code 50)
--
-- Expected UDP payload:
--   Offset 0: System Features                 (4 bytes)
--   Offset 4: High Precision Expanded Ownship (44 bytes)
--
-- Total: 48 bytes
--
-- Assumptions:
--   Network byte order (big-endian)
--   Position, attitude, altitude: IEEE 754 float32
--   Latitude/longitude: raw 48-bit components
--
-- Change UDP_PORT to the actual port.

local UDP_PORT = 8002

local visual = Proto("visualproto", "Visual Protocol")
local f = visual.fields

-- Decode 32-bit SB-1 (signed binary fraction)
-- Units: revolutions
-- Output: degrees
local function decode32SB1ToDegrees(buffer, offset, tree, field)

    local range = buffer(offset, 4)

    -- Read signed 32-bit big-endian integer
    local raw = range:int()

    -- Convert SB-1 to revolutions
    local revolutions = raw / 2147483648.0

    -- Convert revolutions to degrees
    local degrees = revolutions * 180.0

    if degrees < -180.0 then
        degrees = degrees + 360.0
    end

    if degrees > 180.0 then
        degrees = degrees - 360.0
    end

    -- Add decoded value to Wireshark tree
    tree:add(field, degrees)
        :append_text(string.format(
            " (%.8f rev, raw: 0x%08X)",
            revolutions,
            range:uint()))

    return degrees
end

-- Decode 48-bit SB-1 signed binary fraction
-- Units: revolutions
-- Output: degrees
--
-- msb_offset: offset of 32-bit most significant portion
-- lsb_offset: offset of 16-bit least significant portion

local function decode48SB1ToDegrees(buffer, msb_offset, lsb_offset, tree, field)

    -- Read unsigned portions (big-endian)
    local msb = buffer(msb_offset, 4):uint()
    local lsb = buffer(lsb_offset, 2):uint()

    -- Combine into a 48-bit unsigned integer
    -- Lua numbers can represent this exactly (48 < 53 bits)
    local raw = msb * 65536 + lsb

    -- Convert to signed 48-bit two's complement
    if raw >= 140737488355328 then  -- 2^47
        raw = raw - 281474976710656 -- 2^48
    end

    -- Convert SB-1 to revolutions
    local revolutions = raw / 140737488355328.0

    -- Convert revolutions to degrees
    local degrees = revolutions * 180.0

    if degrees < -180.0 then
        degrees = degrees + 360.0
    end

    if degrees > 180.0 then
        degrees = degrees - 360.0
    end

    -- Add decoded value to Wireshark tree
    tree:add(field, degrees)
        :append_text(string.format(
            " (%.12f rev, MSB: 0x%08X, LSB: 0x%04X)",
            revolutions, msb, lsb))

    return degrees
end

-- Decode 32-bit SB23 fixed-point value
-- Units: Feet
-- Resolution: 1/256 feet

local function decode32SB23ToFeet(buffer, offset, tree, field)

    local range = buffer(offset, 4)

    -- Read signed 32-bit big-endian integer
    local raw = range:int()

    -- Convert SB23 to feet
    local altitude_feet = raw / 256.0

    tree:add(field, altitude_feet)
        :append_text(string.format(
            " (raw: 0x%08X)",
            range:uint()))

    return altitude_feet
end

--------------------------------------------------
-- SYSTEM FEATURES
--------------------------------------------------

f.sf_code = ProtoField.uint8(
    "visualproto.sf.code",
    "Data Block Code", base.DEC)

f.sf_raw = ProtoField.uint32(
    "visualproto.sf.raw",
    "Raw System Features", base.HEX)

f.sf_freeze = ProtoField.bool(
    "visualproto.sf.freeze",
    "Visual Freeze", 32, nil, 0x00800000)

f.sf_video = ProtoField.bool(
    "visualproto.sf.video",
    "Video On/Off", 32, nil, 0x00400000)

f.sf_buffer = ProtoField.bool(
    "visualproto.sf.buffer",
    "Host Buffer Save", 32, nil, 0x00200000)

f.sf_scene = ProtoField.bool(
    "visualproto.sf.scene",
    "Type of Generic Scene", 32, nil, 0x00100000)

f.sf_crash = ProtoField.bool(
    "visualproto.sf.crash",
    "Crash Indicator", 32, nil, 0x00080000)

f.sf_persistence = ProtoField.bool(
    "visualproto.sf.persistence",
    "Visual Persistence Reset", 32, nil, 0x00040000)

f.sf_expanded = ProtoField.bool(
    "visualproto.sf.expanded",
    "Expanded Eyepoint Select", 32, nil, 0x00020000)

f.sf_nvg = ProtoField.bool(
    "visualproto.sf.nvg",
    "NVG", 32, nil, 0x00010000)

f.sf_earth = ProtoField.bool(
    "visualproto.sf.earth",
    "Earth Coordinate System Request", 32, nil, 0x00004000)

f.sf_lamp = ProtoField.bool(
    "visualproto.sf.lamp",
    "Lamp On/Off", 32, nil, 0x00002000)

f.sf_morning = ProtoField.uint32(
    "visualproto.sf.morning",
    "Morning Readiness Test",
    base.DEC, nil, 0x00000C00)

f.sf_eyepoint = ProtoField.uint32(
    "visualproto.sf.eyepoint",
    "Eyepoint Select",
    base.DEC, nil, 0x00000300)

--------------------------------------------------
-- HIGH PRECISION EXPANDED OWNSHIP
--------------------------------------------------

f.os_code = ProtoField.uint8(
    "visualproto.os.code",
    "Data Block Code", base.DEC)

f.os_header = ProtoField.uint32(
    "visualproto.os.header",
    "Raw Header", base.HEX)

f.os_mask = ProtoField.uint16(
    "visualproto.os.mask",
    "Device Disable Mask", base.HEX)

-- Device disable mask:
-- Field 1 is bit 15, field 15 is bit 1.
-- Bit 0 is spare.

local device_names = {
    [15] = "IEEE 754 CG",
    [14] = "Wind Screen Wiper",
    [13] = "Wing Tip Strobes",
    [12] = "Anti-Collision Beacon 1",
    [11] = "Weight on Wheels",
    [10] = "Ambient Cockpit Lighting",
    [9]  = "Spare",
    [8]  = "Anti-Collision Beacon 2",
    [7]  = "Anti-Collision Beacon 3",
    [6]  = "Anti-Collision Beacon 4",
    [5]  = "Anti-Collision Beacon 5",
    [4]  = "Anti-Collision Beacon 6",
    [3]  = "Anti-Collision Beacon 7",
    [2]  = "Flare Modes",
    [1]  = "Covert Lighting",
    [0]  = "Spare"
}

local device_fields = {}

for bit = 0, 15 do
    local name = "visualproto.os.device" .. bit

    device_fields[bit] = ProtoField.bool(
        name,
        device_names[bit],
        16,
        nil,
        2 ^ bit)

    f["os_device" .. bit] = device_fields[bit]
end

-- Position

f.os_xcg = ProtoField.float(
    "visualproto.os.xcg", "XCG")

f.os_ycg = ProtoField.float(
    "visualproto.os.ycg", "YCG")

f.os_zcg = ProtoField.float(
    "visualproto.os.zcg", "ZCG")

-- Attitude

f.os_roll = ProtoField.float(
    "visualproto.os.roll", "Roll")

f.os_pitch = ProtoField.float(
    "visualproto.os.pitch", "Pitch")

f.os_heading = ProtoField.float(
    "visualproto.os.heading", "Heading")

-- High precision geodetic coordinates

f.os_latitude = ProtoField.double(
    "visualproto.os.latitude", "Latitude (degrees)")

f.os_longitude = ProtoField.double(
    "visualproto.os.longitude", "Longitude (degrees)")

-- Altitude

f.os_altitude = ProtoField.double(
    "visualproto.os.altitude",
    "Altitude (feet)")

--------------------------------------------------
-- SYSTEM FEATURES DECODER
--------------------------------------------------

local function decode_system_features(buffer, tree, offset)

    if buffer:len() < offset + 4 then
        return 0
    end

    local word = buffer(offset, 4)

    local t = tree:add(
        visual,
        word,
        "System Features (Code 0)")

    t:add(f.sf_raw, word)
    t:add(f.sf_code, buffer(offset, 1))

    t:add(f.sf_freeze, word)
    t:add(f.sf_video, word)
    t:add(f.sf_buffer, word)
    t:add(f.sf_scene, word)
    t:add(f.sf_crash, word)
    t:add(f.sf_persistence, word)
    t:add(f.sf_expanded, word)
    t:add(f.sf_nvg, word)

    t:add(f.sf_earth, word)
    t:add(f.sf_lamp, word)
    t:add(f.sf_morning, word)
    t:add(f.sf_eyepoint, word)

    return 4
end

--------------------------------------------------
-- HIGH PRECISION EXPANDED OWNSHIP DECODER
--------------------------------------------------

local function decode_ownship(buffer, tree, offset)

    local BLOCK_SIZE = 44

    if buffer:len() < offset + BLOCK_SIZE then
        return 0
    end

    local t = tree:add(
        visual,
        buffer(offset, BLOCK_SIZE),
        "High Precision Expanded Ownship (Code 50)")

    -- Header
    t:add(f.os_header, buffer(offset, 4))
    t:add(f.os_code, buffer(offset, 1))

    -- Device disable mask
    local mask_range = buffer(offset + 2, 2)

    local mask_tree = t:add(
        f.os_mask, mask_range)

    for bit = 15, 0, -1 do
        mask_tree:add(
            device_fields[bit],
            mask_range)
    end

    -- Position
    t:add(f.os_xcg, buffer(offset + 8, 4))
    t:add(f.os_ycg, buffer(offset + 12, 4))
    t:add(f.os_zcg, buffer(offset + 16, 4))

    -- Attitude
    local roll = decode32SB1ToDegrees(buffer, offset + 20, t, f.os_roll)
    local pitch = decode32SB1ToDegrees(buffer, offset + 24, t, f.os_pitch)
    local heading = decode32SB1ToDegrees(buffer, offset + 28, t, f.os_heading)

    -- Latitude: 48-bit SB-1
    -- MSB at offset +32, LSB at offset +36
    local latitude = decode48SB1ToDegrees(
        buffer,
        offset + 32,
        offset + 36,
        t,
        f.os_latitude)

    -- Longitude: 48-bit SB-1
    -- MSB at offset +40, LSB at offset +38
    local longitude = decode48SB1ToDegrees(
        buffer,
        offset + 40,
        offset + 38,
        t,
        f.os_longitude)

    -- Altitude
    local altitude = decode32SB23ToFeet(buffer, offset + 44, t, f.os_altitude)

    return BLOCK_SIZE
end

--------------------------------------------------
-- MAIN DISSECTOR
--------------------------------------------------

function visual.dissector(buffer, pinfo, tree)

    local length = buffer:len()

    if length < 4 then
        return 0
    end

    pinfo.cols.protocol = "VISUAL"

    local root = tree:add(
        visual,
        buffer(),
        "Visual Protocol")

    local offset = 0

    -- First block: System Features
    if buffer(offset, 1):uint() == 0 then

        local consumed =
            decode_system_features(buffer, root, offset)

        offset = offset + consumed
    else
        root:add_expert_info(
            PI_MALFORMED,
            PI_WARN,
            "Expected System Features block (code 0)")
        return length
    end

    -- Second block: High Precision Expanded Ownship
    if length >= offset + 4 then

        local code = buffer(offset, 1):uint()

        if code == 50 then

            local consumed =
                decode_ownship(buffer, root, offset)

            offset = offset + consumed
        end
    end

    -- Any remaining packet data
    if offset < length then
        root:add(
            buffer(offset, length - offset),
            "Remaining Data")
    end

    pinfo.cols.info = string.format(
        "Visual Protocol - %d bytes", length)

    return length
end

--------------------------------------------------
-- REGISTER UDP PORT
--------------------------------------------------

DissectorTable.get("udp.port"):add(
    UDP_PORT, visual)

