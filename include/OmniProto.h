#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>

// Wire protocol shared by S3 (OmniController) and C6 (OmniControllerC6Firmware).
// This file is vendored verbatim into the C6 project. When the protocol changes,
// bump OMNI_PROTO_VERSION and update both copies in lockstep — the CTRL `hello`
// handshake refuses to bring the link up if the versions disagree.
//
// Frame format (placeholder for M-α; concrete encode/decode lands in M-γ):
//
//   +0  MAGIC    1 B   0xA5
//   +1  CH       1 B   channel id (see OmniChannel)
//   +2  FLAGS    1 B   bit0 FRAG_MORE | bit1 ACK_REQ | bit2 ACK
//   +3  LEN      2 B   payload length, little-endian, max 1024
//   +5  SEQ      2 B   little-endian sequence number
//   +7  PAYLOAD  LEN bytes
//   +7+LEN CRC16 2 B   CRC16-CCITT-FALSE over CH..PAYLOAD inclusive
//
// Encoding inside payload:
//   CTRL/THREAD/ZIGBEE/MATTER  → UTF-8 JSON
//   LOG                         → UTF-8 text
//   OTA                         → raw bytes

#ifndef OMNI_PROTO_VERSION
#define OMNI_PROTO_VERSION 1
#endif

namespace omni {

constexpr uint8_t kFrameMagic = 0xA5;
constexpr uint16_t kMaxPayload = 1024;
constexpr uint16_t kMaxFrame   = 1033;  // header(7) + payload(1024) + crc(2)

enum class Channel : uint8_t {
    Ctrl   = 0x00,
    Log    = 0x01,
    Ota    = 0x02,
    Thread = 0x10,
    Zigbee = 0x11,
    Matter = 0x12,
    Raw1   = 0x20,  // reserved (future: CC1101)
    Raw2   = 0x21,  // reserved (future: IR)
};

enum FrameFlag : uint8_t {
    FlagFragMore = 0x01,
    FlagAckReq   = 0x02,
    FlagAck      = 0x04,
};

// CTRL channel ops (JSON `op` field). Listed here so both sides agree on names;
// handlers land in M-γ (link bring-up), M-δ (OTA), M-θ (Thread / Matter),
// M-κ (radio roles).
namespace ctrl {
constexpr const char* kOpHello          = "hello";
constexpr const char* kOpHelloAck       = "hello_ack";
constexpr const char* kOpPing           = "ping";
constexpr const char* kOpPong           = "pong";
constexpr const char* kOpOtaBegin       = "ota_begin";
constexpr const char* kOpOtaBeginAck    = "ota_begin_ack";
constexpr const char* kOpOtaChunkAck    = "ota_chunk_ack";
constexpr const char* kOpOtaEnd         = "ota_end";
constexpr const char* kOpOtaStatus      = "ota_status";
constexpr const char* kOpReset          = "reset";
constexpr const char* kOpBootloader     = "bootloader";
constexpr const char* kOpResync         = "resync";
constexpr const char* kOpLogSubscribe   = "log_subscribe";

// Thread (M-θ Push A). thread_status is request-only from S3; the C6
// replies with thread_status_reply containing role, stack state, dataset
// presence. Network bring-up ops (thread_enable / thread_disable) land
// in Push B once we're ready to commit datasets.
constexpr const char* kOpThreadStatus      = "thread_status";
constexpr const char* kOpThreadStatusReply = "thread_status_reply";

// Dynamic radio roles (M-κ). The C6 carries WiFi 6 + BLE 5 + 802.15.4 on a
// single 2.4 GHz front-end. Default role is 802154_only; the S3 can ask the
// C6 to take over WiFi or BLE workloads when the S3's own radios are busy
// (e.g. S3 running a BLE GATT server for provisioning).
constexpr const char* kOpRadioRoleSet     = "radio_role_set";       // {"mode":"..."}
constexpr const char* kOpRadioRoleGet     = "radio_role_get";
constexpr const char* kOpRadioRoleChanged = "radio_role_changed";   // C6→S3 confirmation
constexpr const char* kOpBleAdvert        = "ble_advert";           // C6→S3 forwarded BLE adv
constexpr const char* kOpWifiScanResult   = "wifi_scan_result";     // C6→S3 forwarded scan hit
}  // namespace ctrl

namespace radio {
constexpr const char* kMode802154Only      = "802154_only";
constexpr const char* kModeWifi802154Coex  = "wifi_802154_coex";
constexpr const char* kModeBle802154Coex   = "ble_802154_coex";
constexpr const char* kModeWifiOnly        = "wifi_only";
constexpr const char* kModeBleOnly         = "ble_only";
}  // namespace radio

// Fixed transaction size for the SPI link. Both sides clock exactly this many
// bytes per transaction. Frames smaller than this are zero-padded after the
// CRC (the LEN field tells the receiver how many payload bytes are valid).
// Sized to fit one full 1024-byte payload frame plus header + CRC plus a few
// spare bytes.
constexpr uint16_t kSpiTransactionBytes = 1040;

// CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xorout).
// Inline so both S3 and C6 builds can use it without linking.
inline uint16_t crc16CcittFalse(const uint8_t* data, size_t len, uint16_t init = 0xFFFF) {
    uint16_t crc = init;
    while (len--) {
        crc ^= (static_cast<uint16_t>(*data++) << 8);
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

// Encode a frame into a buffer of size kSpiTransactionBytes. Returns the
// total frame length (header+payload+crc) on success, or 0 on failure
// (payload too large, output too small, etc.). The buffer is padded with
// 0xFF after the frame content so the SPI master/slave can clock fixed-size
// transactions while still carrying variable-length frames.
inline size_t encodeFrame(uint8_t* out, size_t outSize,
                          Channel channel, uint8_t flags, uint16_t seq,
                          const uint8_t* payload, uint16_t payloadLen) {
    if (payloadLen > kMaxPayload) return 0;
    if (outSize < kSpiTransactionBytes) return 0;
    out[0] = kFrameMagic;
    out[1] = static_cast<uint8_t>(channel);
    out[2] = flags;
    out[3] = static_cast<uint8_t>(payloadLen & 0xFF);
    out[4] = static_cast<uint8_t>((payloadLen >> 8) & 0xFF);
    out[5] = static_cast<uint8_t>(seq & 0xFF);
    out[6] = static_cast<uint8_t>((seq >> 8) & 0xFF);
    if (payloadLen && payload) {
        for (uint16_t i = 0; i < payloadLen; i++) out[7 + i] = payload[i];
    }
    uint16_t crc = crc16CcittFalse(out + 1, 6 + payloadLen);  // CH..PAYLOAD
    out[7 + payloadLen]     = static_cast<uint8_t>(crc & 0xFF);
    out[7 + payloadLen + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    size_t total = 9 + payloadLen;
    for (size_t i = total; i < kSpiTransactionBytes; i++) out[i] = 0xFF;
    return total;
}

struct DecodedFrame {
    Channel  channel;
    uint8_t  flags;
    uint16_t seq;
    uint16_t payloadLen;
    const uint8_t* payload;  // points into the source buffer
};

// Decode a frame from a buffer. Returns true if the frame is valid (magic
// matches, length sane, CRC verifies). On success, `out.payload` points into
// `in` — caller must keep `in` valid while using `out.payload`.
inline bool decodeFrame(const uint8_t* in, size_t inSize, DecodedFrame& out) {
    if (inSize < 9) return false;
    if (in[0] != kFrameMagic) return false;
    uint16_t len = static_cast<uint16_t>(in[3]) | (static_cast<uint16_t>(in[4]) << 8);
    if (len > kMaxPayload) return false;
    if (inSize < static_cast<size_t>(9 + len)) return false;
    uint16_t crcReceived = static_cast<uint16_t>(in[7 + len])
                         | (static_cast<uint16_t>(in[7 + len + 1]) << 8);
    uint16_t crcCalc = crc16CcittFalse(in + 1, 6 + len);
    if (crcReceived != crcCalc) return false;
    out.channel    = static_cast<Channel>(in[1]);
    out.flags      = in[2];
    out.seq        = static_cast<uint16_t>(in[5]) | (static_cast<uint16_t>(in[6]) << 8);
    out.payloadLen = len;
    out.payload    = (len > 0) ? (in + 7) : nullptr;
    return true;
}

}  // namespace omni
