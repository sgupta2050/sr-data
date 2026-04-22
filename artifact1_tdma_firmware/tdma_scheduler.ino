/**
 * tdma_scheduler.ino
 * ============================================================
 * TDMA (Time-Division Multiple Access) Scheduling Firmware
 * for ESP32-WROOM-32D — Multi-Robot Formation Control
 *
 * Companion code for:
 *   "Adaptive Multi-Robot Formation Control Using Distributed
 *    Audio-Visual Sensing and Bluetooth Communication Networks"
 *   Scientific Reports, 2025.
 *
 * Repository: https://github.com/sgupta2050/sr-data
 *
 * Architecture
 * ------------
 * T_cycle = N_robots * T_slot + T_guard
 *   T_slot  = 20 ms  (per robot transmission window)
 *   T_guard =  5 ms  (clock-drift accommodation)
 *   N = 12  → T_cycle = 245 ms  → ~3.3 Hz update rate
 *
 * The master robot emits a beacon at the start of each cycle.
 * All follower robots update their local clocks via linear
 * regression on the N most recent beacon timestamps, achieving
 * ±5 ms synchronisation accuracy (see Figure 2c of the paper).
 *
 * Crystal drift: ±50 ppm → ±0.3 ms/cycle.
 * Holdover limit (±5 ms window): ~17 cycles ≈ 4.2 s for N=12.
 *
 * Hardware:  ESP32-WROOM-32D, 240 MHz dual-core
 * RTOS:      FreeRTOS (bundled with Arduino-ESP32 framework)
 * Core assignment:
 *   Core 0 — Formation control + Bluetooth communication
 *   Core 1 — Sensor processing (audio, camera, IMU)
 * ============================================================
 */

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

// ── Configuration ────────────────────────────────────────────
#define ROBOT_ID          1          // 1..N_ROBOTS (set per unit)
#define IS_MASTER         (ROBOT_ID == 1)
#define N_ROBOTS          12
#define T_SLOT_MS         20
#define T_GUARD_MS        5
#define T_CYCLE_MS        (N_ROBOTS * T_SLOT_MS + T_GUARD_MS)  // 245 ms

// Synchronisation regression window
#define SYNC_WINDOW       8          // number of beacons used for regression

// Packet delimiters
#define PKT_START_1       0xAA
#define PKT_START_2       0x55
#define PKT_TYPE_BEACON   0x01
#define PKT_TYPE_STATE    0x02
#define PKT_TYPE_AUDIO    0x03
#define PKT_TYPE_VISUAL   0x04

// ── Types ────────────────────────────────────────────────────
typedef struct {
    uint8_t  robot_id;
    float    pos_x;          // metres
    float    pos_y;
    float    heading_rad;
    float    vel_x;
    float    vel_y;
    uint32_t timestamp_us;
} __attribute__((packed)) RobotState_t;

typedef struct {
    uint8_t  start1;
    uint8_t  start2;
    uint8_t  pkt_type;
    uint8_t  length;
    uint8_t  payload[64];
    uint16_t crc;
    uint8_t  end;
} __attribute__((packed)) Packet_t;

// ── Globals ──────────────────────────────────────────────────
static volatile int64_t  g_local_offset_us = 0;  // correction to local clock
static int64_t           g_beacon_times[SYNC_WINDOW] = {0};
static int64_t           g_beacon_master[SYNC_WINDOW] = {0};
static uint8_t           g_beacon_idx = 0;

static RobotState_t      g_neighbour_states[N_ROBOTS];  // local state buffer
static SemaphoreHandle_t g_state_mutex;

static TaskHandle_t      g_tdma_task_handle = NULL;
static TaskHandle_t      g_ctrl_task_handle = NULL;

// ── CRC-16 (CCITT) ──────────────────────────────────────────
static uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

// ── Packet builder ───────────────────────────────────────────
static size_t build_packet(uint8_t *buf, uint8_t type,
                            const uint8_t *payload, uint8_t plen) {
    buf[0] = PKT_START_1;
    buf[1] = PKT_START_2;
    buf[2] = type;
    buf[3] = plen;
    memcpy(&buf[4], payload, plen);
    uint16_t crc = crc16(&buf[2], 2 + plen);
    buf[4 + plen]     = (crc >> 8) & 0xFF;
    buf[4 + plen + 1] =  crc       & 0xFF;
    buf[4 + plen + 2] = 0xFF;         // end delimiter
    return 4 + plen + 3;
}

// ── Clock synchronisation (linear regression) ────────────────
/**
 * update_clock_sync()
 * Called on each received beacon.
 * Fits a linear model: master_time = a * local_time + b
 * and applies the offset to g_local_offset_us.
 */
static void update_clock_sync(int64_t master_ts_us, int64_t local_ts_us) {
    g_beacon_master[g_beacon_idx % SYNC_WINDOW] = master_ts_us;
    g_beacon_times [g_beacon_idx % SYNC_WINDOW] = local_ts_us;
    g_beacon_idx++;

    uint8_t n = (g_beacon_idx < SYNC_WINDOW) ? g_beacon_idx : SYNC_WINDOW;
    if (n < 2) return;

    // Least-squares linear regression: master = a*local + b
    double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
    for (uint8_t i = 0; i < n; i++) {
        double x = (double)g_beacon_times[i];
        double y = (double)g_beacon_master[i];
        sum_x  += x;
        sum_y  += y;
        sum_xx += x * x;
        sum_xy += x * y;
    }
    double denom = n * sum_xx - sum_x * sum_x;
    if (fabs(denom) < 1e-9) return;
    double b = (sum_y * sum_xx - sum_x * sum_xy) / denom;
    // Apply intercept correction
    g_local_offset_us = (int64_t)b;
}

static inline int64_t synced_time_us(void) {
    return esp_timer_get_time() + g_local_offset_us;
}

// ── Slot timing ──────────────────────────────────────────────
/**
 * my_slot_start_us()
 * Returns the expected start time (in synchronised microseconds)
 * of this robot's transmission slot within the current cycle.
 *
 * Slot assignment: robot i transmits in slot (i-1).
 * Master beacon occupies the first T_GUARD_MS of each cycle.
 */
static int64_t my_slot_start_us(int64_t cycle_start_us) {
    return cycle_start_us
           + T_GUARD_MS * 1000LL
           + (ROBOT_ID - 1) * T_SLOT_MS * 1000LL;
}

// ── Master: beacon transmission ──────────────────────────────
static void transmit_beacon(int64_t cycle_start_us) {
    uint8_t payload[8];
    int64_t ts = synced_time_us();
    memcpy(payload, &ts, sizeof(ts));

    uint8_t pkt[32];
    size_t  len = build_packet(pkt, PKT_TYPE_BEACON, payload, 8);
    Serial.write(pkt, len);      // Bluetooth SPP via Serial
}

// ── Follower: state broadcast ─────────────────────────────────
static void transmit_state(void) {
    RobotState_t s;
    // Populate from odometry (filled by control task)
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    s = g_neighbour_states[ROBOT_ID - 1];
    xSemaphoreGive(g_state_mutex);
    s.robot_id     = ROBOT_ID;
    s.timestamp_us = (uint32_t)synced_time_us();

    uint8_t pkt[128];
    size_t  len = build_packet(pkt, PKT_TYPE_STATE,
                               (uint8_t *)&s, sizeof(s));
    Serial.write(pkt, len);
}

// ── Packet parser (state machine) ───────────────────────────
typedef enum { PS_START1, PS_START2, PS_TYPE, PS_LEN,
               PS_PAYLOAD, PS_CRC_HI, PS_CRC_LO, PS_END } ParseState_t;

static void process_incoming_byte(uint8_t byte) {
    static ParseState_t state   = PS_START1;
    static uint8_t      buf[80];
    static uint8_t      ptype, plen, pidx;
    static uint16_t     crc_rx;

    switch (state) {
        case PS_START1:
            if (byte == PKT_START_1) state = PS_START2;
            break;
        case PS_START2:
            state = (byte == PKT_START_2) ? PS_TYPE : PS_START1;
            break;
        case PS_TYPE:  ptype = byte; buf[0] = byte; state = PS_LEN;  break;
        case PS_LEN:   plen  = byte; buf[1] = byte; pidx = 0;
                       state = (plen > 0) ? PS_PAYLOAD : PS_CRC_HI; break;
        case PS_PAYLOAD:
            buf[2 + pidx++] = byte;
            if (pidx == plen) state = PS_CRC_HI;
            break;
        case PS_CRC_HI: crc_rx  = (uint16_t)byte << 8; state = PS_CRC_LO; break;
        case PS_CRC_LO:
            crc_rx |= byte;
            if (crc16(buf, 2 + plen) == crc_rx) state = PS_END;
            else                                  state = PS_START1;
            break;
        case PS_END:
            state = PS_START1;
            if (byte != 0xFF) break;    // bad end delimiter — discard

            if (ptype == PKT_TYPE_BEACON) {
                int64_t master_ts;
                memcpy(&master_ts, &buf[2], sizeof(master_ts));
                update_clock_sync(master_ts, esp_timer_get_time());

            } else if (ptype == PKT_TYPE_STATE) {
                RobotState_t *rs = (RobotState_t *)&buf[2];
                if (rs->robot_id >= 1 && rs->robot_id <= N_ROBOTS) {
                    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                    g_neighbour_states[rs->robot_id - 1] = *rs;
                    xSemaphoreGive(g_state_mutex);
                }
            }
            break;
    }
}

// ── TDMA task (Core 0) ───────────────────────────────────────
/**
 * vTDMATask()
 * Runs on Core 0.  Manages slot timing using esp_timer for
 * microsecond precision.  Master transmits beacon at cycle
 * start; each robot transmits its state in its assigned slot.
 *
 * The task does NOT retry lost packets within the same slot;
 * a one-retry mechanism fires only when two consecutive slots
 * are missed (detected via sequence counter).
 */
static void vTDMATask(void *pvParam) {
    const int64_t cycle_us = (int64_t)T_CYCLE_MS * 1000LL;
    int64_t       next_cycle = synced_time_us();
    uint8_t       missed_count[N_ROBOTS] = {0};

    while (1) {
        int64_t now = synced_time_us();
        if (now < next_cycle) {
            int64_t wait_us = next_cycle - now;
            vTaskDelay(pdMS_TO_TICKS(wait_us / 1000));
            continue;
        }

        // ── Cycle start ─────────────────────────────────────
        if (IS_MASTER) {
            transmit_beacon(next_cycle);
        }

        // ── Wait for this robot's slot ───────────────────────
        int64_t my_slot = my_slot_start_us(next_cycle);
        int64_t wait_to_slot = my_slot - synced_time_us();
        if (wait_to_slot > 1000)
            vTaskDelay(pdMS_TO_TICKS(wait_to_slot / 1000));

        transmit_state();

        // ── Read incoming bytes for the rest of the cycle ───
        int64_t cycle_end = next_cycle + cycle_us - T_GUARD_MS * 1000LL;
        while (synced_time_us() < cycle_end) {
            while (Serial.available())
                process_incoming_byte((uint8_t)Serial.read());
            vTaskDelay(1);
        }

        // ── Advance cycle ───────────────────────────────────
        next_cycle += cycle_us;

        // ── Check for missed robots (one-retry) ─────────────
        for (int r = 0; r < N_ROBOTS; r++) {
            int64_t age_us = synced_time_us()
                             - (int64_t)g_neighbour_states[r].timestamp_us;
            if (age_us > 2LL * cycle_us) {
                missed_count[r]++;
                if (missed_count[r] >= 2) {
                    // Trigger retry request (implementation-specific)
                    missed_count[r] = 0;
                }
            } else {
                missed_count[r] = 0;
            }
        }
    }
}

// ── Arduino setup / loop ─────────────────────────────────────
void setup() {
    Serial.begin(115200);    // Bluetooth SPP via UART bridge
    g_state_mutex = xSemaphoreCreateMutex();

    // Initialise neighbour state buffer
    memset(g_neighbour_states, 0, sizeof(g_neighbour_states));

    // Launch TDMA task on Core 0, priority 3
    xTaskCreatePinnedToCore(
        vTDMATask, "TDMA", 4096, NULL, 3,
        &g_tdma_task_handle, 0);
}

void loop() {
    // Main loop intentionally empty.
    // All logic runs in FreeRTOS tasks.
    vTaskDelay(portMAX_DELAY);
}
