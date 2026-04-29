#pragma once

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
// handlers land in M-γ (link bring-up), M-δ (OTA), M-κ (radio roles).
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

}  // namespace omni
