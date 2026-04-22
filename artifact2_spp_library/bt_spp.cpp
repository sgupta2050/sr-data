/**
 * bt_spp.cpp
 * ============================================================
 * Bluetooth Classic SPP Communication Library — Implementation
 * ESP32-WROOM-32D — Multi-Robot Formation Control
 *
 * See bt_spp.h for API documentation.
 * Repository: https://github.com/sgupta2050/sr-data
 * ============================================================
 */

#include "bt_spp.h"
#include "BluetoothSerial.h"   // ESP32 Arduino Bluetooth Classic
#include "esp_timer.h"
#include <string.h>

// ── Internal state ────────────────────────────────────────────
static BluetoothSerial  g_bt;
static BtSppCallback_t  g_callback = nullptr;

// ── CRC-16/CCITT ─────────────────────────────────────────────
static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

// ── Packet framing ────────────────────────────────────────────
static size_t frame_packet(uint8_t *out, uint8_t type,
                            const uint8_t *payload, uint8_t plen) {
    out[0] = 0xAA;
    out[1] = 0x55;
    out[2] = type;
    out[3] = plen;
    memcpy(&out[4], payload, plen);
    uint16_t crc = crc16_ccitt(&out[2], 2 + plen);
    out[4 + plen]     = (crc >> 8) & 0xFF;
    out[4 + plen + 1] =  crc       & 0xFF;
    out[4 + plen + 2] = 0xFF;
    return (size_t)(4 + plen + 3);
}

// ── Public: init ──────────────────────────────────────────────
void bt_spp_init(const char *device_name) {
    g_bt.begin(device_name);
}

// ── Public: send ──────────────────────────────────────────────
int bt_spp_send(uint8_t type, const uint8_t *payload, uint8_t plen) {
    if (plen > PKT_MAX_PAYLOAD) return -1;
    uint8_t buf[PKT_BUF_SIZE];
    size_t  len = frame_packet(buf, type, payload, plen);
    return (int)g_bt.write(buf, len);
}

// ── Packet parser state machine ───────────────────────────────
typedef enum {
    PS_S1, PS_S2, PS_TYPE, PS_LEN,
    PS_PAYLOAD, PS_CRC_HI, PS_CRC_LO, PS_END
} ParseState_t;

static void parser_feed(uint8_t byte) {
    static ParseState_t state = PS_S1;
    static uint8_t  hdr[2];           // [TYPE, LEN]
    static uint8_t  pbuf[PKT_MAX_PAYLOAD];
    static uint8_t  pidx;
    static uint16_t crc_rx;

    switch (state) {
        case PS_S1:
            if (byte == 0xAA) state = PS_S2;
            break;
        case PS_S2:
            state = (byte == 0x55) ? PS_TYPE : PS_S1;
            break;
        case PS_TYPE:
            hdr[0] = byte; state = PS_LEN;
            break;
        case PS_LEN:
            hdr[1] = byte; pidx = 0;
            state = (byte > 0) ? PS_PAYLOAD : PS_CRC_HI;
            break;
        case PS_PAYLOAD:
            if (pidx < PKT_MAX_PAYLOAD) pbuf[pidx++] = byte;
            if (pidx == hdr[1]) state = PS_CRC_HI;
            break;
        case PS_CRC_HI:
            crc_rx = (uint16_t)byte << 8; state = PS_CRC_LO;
            break;
        case PS_CRC_LO: {
            crc_rx |= byte;
            // Verify CRC over [TYPE, LEN, PAYLOAD]
            uint8_t tmp[2 + PKT_MAX_PAYLOAD];
            tmp[0] = hdr[0]; tmp[1] = hdr[1];
            memcpy(&tmp[2], pbuf, hdr[1]);
            if (crc16_ccitt(tmp, 2 + hdr[1]) == crc_rx)
                state = PS_END;
            else
                state = PS_S1;
            break;
        }
        case PS_END:
            state = PS_S1;
            if (byte == 0xFF && g_callback)
                g_callback(hdr[0], pbuf, hdr[1]);
            break;
    }
}

// ── Public: receive tick ──────────────────────────────────────
void bt_spp_recv_tick(void) {
    while (g_bt.available())
        parser_feed((uint8_t)g_bt.read());
}

// ── Public: callback registration ───────────────────────────
void bt_spp_register_callback(BtSppCallback_t cb) {
    g_callback = cb;
}

// ── Public: connection status ────────────────────────────────
bool bt_spp_connected(void) {
    return g_bt.connected();
}

// ── Public: convenience wrappers ─────────────────────────────
int bt_spp_send_state(const RobotState_t *state) {
    return bt_spp_send(PKT_STATE,
                       (const uint8_t *)state,
                       (uint8_t)sizeof(RobotState_t));
}

int bt_spp_send_beacon(int64_t synced_us) {
    uint8_t payload[8];
    memcpy(payload, &synced_us, 8);
    return bt_spp_send(PKT_BEACON, payload, 8);
}
