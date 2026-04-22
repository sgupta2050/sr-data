/**
 * bt_spp.h
 * ============================================================
 * Bluetooth Classic SPP Communication Library
 * ESP32-WROOM-32D — Multi-Robot Formation Control
 *
 * Companion code for:
 *   "Adaptive Multi-Robot Formation Control Using Distributed
 *    Audio-Visual Sensing and Bluetooth Communication Networks"
 *   Scientific Reports, 2025.
 *
 * Repository: https://github.com/sgupta2050/sr-data
 *
 * Packet structure (Section 4.2 of the paper):
 *   [0xAA] [0x55] [TYPE] [LEN] [PAYLOAD (≤64 B)] [CRC_HI] [CRC_LO] [0xFF]
 *
 * Packet types:
 *   PKT_BEACON  — master clock synchronisation (8 bytes)
 *   PKT_STATE   — robot state: pos, vel, heading (sizeof RobotState_t)
 *   PKT_AUDIO   — µ-law compressed audio descriptor
 *   PKT_VISUAL  — ArUco marker: id + pose + confidence (24 bytes/feature)
 *
 * Baud rate: 115,200 bps (Bluetooth Classic SPP)
 * Max simultaneous SPP connections per ESP32: 7
 * For N=8–12, a two-tier multi-hop relay topology is used
 * (see Section 5.2 of the paper and tdma_scheduler.ino).
 * ============================================================
 */

#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

// ── Packet type codes ────────────────────────────────────────
#define PKT_BEACON   0x01
#define PKT_STATE    0x02
#define PKT_AUDIO    0x03
#define PKT_VISUAL   0x04

// ── Maximum payload sizes ────────────────────────────────────
#define PKT_MAX_PAYLOAD   64     // bytes — fits in single SPP frame
#define PKT_VISUAL_FEAT   24     // bytes per ArUco feature descriptor
#define PKT_MAX_VISUAL    8      // max features per slot

// ── Packet overhead ──────────────────────────────────────────
// Header: START1 START2 TYPE LEN  = 4 bytes
// Trailer: CRC_HI CRC_LO END     = 3 bytes
#define PKT_OVERHEAD      7

// ── Worst-case packet buffer size ────────────────────────────
#define PKT_BUF_SIZE      (PKT_MAX_PAYLOAD + PKT_OVERHEAD)

// ── Robot state payload (control packets, ≤64 bytes) ─────────
typedef struct {
    uint8_t  robot_id;
    float    pos_x;          // m
    float    pos_y;          // m
    float    heading_rad;    // rad
    float    vel_x;          // m/s
    float    vel_y;          // m/s
    uint32_t timestamp_us;   // synchronised clock, µs
} __attribute__((packed)) RobotState_t;
// sizeof(RobotState_t) = 25 bytes — well within 64-byte limit

// ── Audio payload: µ-law compressed descriptor ───────────────
// µ-law compression (Eq. 2 in paper):
//   y[n] = sign(x[n]) * floor(255 * log(1 + µ|x[n]|) / log(1 + µ))
//   µ = 255 → ~42 dB dynamic range, 50% bandwidth reduction
typedef struct {
    uint8_t  robot_id;
    uint16_t ste_energy;     // short-time energy (16-bit scaled)
    uint8_t  event_flag;     // 1 if STE > 3 × background
    uint8_t  compressed[28]; // µ-law audio descriptor bytes
} __attribute__((packed)) AudioPayload_t;

// ── Visual payload: ArUco feature descriptor (24 bytes each) ─
typedef struct {
    uint8_t  robot_id;
    uint8_t  marker_id;
    float    rel_x;          // m, relative position
    float    rel_y;          // m
    float    rel_heading;    // rad
    uint8_t  confidence;     // 0–255
    uint8_t  _pad[2];
} __attribute__((packed)) VisualFeature_t;
// sizeof(VisualFeature_t) = 16 bytes; up to 8 per slot (Section 5.2)

// ── Public API ───────────────────────────────────────────────

/**
 * bt_spp_init()
 * Initialise Bluetooth Classic SPP on the given Serial port.
 * Call once in setup() before any other bt_spp_* functions.
 *
 * @param device_name  BT device name (e.g. "Robot_01")
 */
void bt_spp_init(const char *device_name);

/**
 * bt_spp_send()
 * Build and transmit a framed packet.
 *
 * @param type     Packet type (PKT_BEACON / PKT_STATE / …)
 * @param payload  Pointer to payload bytes
 * @param plen     Payload length (≤ PKT_MAX_PAYLOAD)
 * @return         Number of bytes written, or -1 on error
 */
int bt_spp_send(uint8_t type, const uint8_t *payload, uint8_t plen);

/**
 * bt_spp_recv_tick()
 * Non-blocking receive poll — call frequently (e.g. every 1 ms).
 * Feeds available bytes into the state-machine parser.
 * Completed, CRC-verified packets are dispatched to the
 * registered callback (see bt_spp_register_callback).
 */
void bt_spp_recv_tick(void);

/**
 * Callback type for received packets.
 * @param type     Packet type code
 * @param payload  Pointer to payload (valid only during callback)
 * @param plen     Payload length
 */
typedef void (*BtSppCallback_t)(uint8_t type,
                                 const uint8_t *payload,
                                 uint8_t plen);

/**
 * bt_spp_register_callback()
 * Register a function to be called when a valid packet arrives.
 */
void bt_spp_register_callback(BtSppCallback_t cb);

/**
 * bt_spp_connected()
 * @return true if at least one SPP peer is connected.
 */
bool bt_spp_connected(void);

/**
 * bt_spp_send_state()
 * Convenience wrapper: serialise a RobotState_t and transmit.
 */
int bt_spp_send_state(const RobotState_t *state);

/**
 * bt_spp_send_beacon()
 * Convenience wrapper: transmit master beacon with current
 * synchronised timestamp.
 */
int bt_spp_send_beacon(int64_t synced_us);
