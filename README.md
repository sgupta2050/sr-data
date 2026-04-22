# sr-data

**Companion repository for:**

> Gupta, S., Mamodiya, U., Kalam, A., & Ali, T. (2025).  
> *Adaptive Multi-Robot Formation Control Using Distributed Audio-Visual Sensing and Bluetooth Communication Networks.*  
> *Scientific Reports.*

Repository: https://github.com/sgupta2050/sr-data

---

## Contents

This repository contains four verifiable hardware implementation artifacts, provided in response to Editor Comment 3 (minor revision, Submission ID 338d3c48-6766-4eb6-a4d5-fbe242eb3762):

| # | Artifact | Description |
|---|----------|-------------|
| 1 | [`artifact1_tdma_firmware/`](artifact1_tdma_firmware/) | TDMA scheduling firmware for ESP32 (Arduino-compatible) |
| 2 | [`artifact2_spp_library/`](artifact2_spp_library/) | Bluetooth Classic SPP communication library |
| 3 | [`artifact3_formation_control/`](artifact3_formation_control/) | Distributed consensus formation control algorithm (Eqs. 3–7) |
| 4 | [`artifact4_sync_data/`](artifact4_sync_data/) | Raw time-synchronization data underlying Figure 2(c) |

---

## Artifact 1 — TDMA Scheduling Firmware

**File:** `artifact1_tdma_firmware/tdma_scheduler.ino`  
**Platform:** ESP32-WROOM-32D, Arduino-ESP32 framework, FreeRTOS  

Implements the Time-Division Multiple Access protocol described in Section 4.2 and Section 5.2 of the paper:

- `T_slot = 20 ms`, `T_guard = 5 ms`
- `T_cycle = N × 20 ms + 5 ms` (e.g. 245 ms for N = 12 → ~3.3 Hz)
- Master beacon emission and linear-regression clock synchronisation (±5 ms accuracy)
- State-machine packet parser with CRC-16/CCITT verification
- One-retry mechanism triggered only after two consecutive missed slots
- Crystal drift: ±50 ppm → ±0.3 ms/cycle → ~4.2 s holdover for N = 12

**Dependencies:** Arduino-ESP32 (≥2.0), FreeRTOS (bundled)

---

## Artifact 2 — Bluetooth SPP Communication Library

**Files:** `artifact2_spp_library/bt_spp.h`, `bt_spp.cpp`  
**Platform:** ESP32-WROOM-32D, 115,200 bps Bluetooth Classic SPP  

Implements the custom packet protocol described in Section 4.2:

```
[0xAA] [0x55] [TYPE] [LEN] [PAYLOAD ≤64 B] [CRC_HI] [CRC_LO] [0xFF]
```

Packet types: `PKT_BEACON` (clock sync), `PKT_STATE` (robot pose/velocity, 25 bytes), `PKT_AUDIO` (µ-law compressed), `PKT_VISUAL` (ArUco descriptor, 24 bytes/feature).

**Key structs:** `RobotState_t`, `AudioPayload_t`, `VisualFeature_t`  
**Dependencies:** `BluetoothSerial` (Arduino-ESP32 built-in)

---

## Artifact 3 — Formation Control Algorithm

**Files:** `artifact3_formation_control/formation_control.h`, `formation_control.cpp`  
**Language:** C++ (Arduino-ESP32)

Implements Equations 3–7 from Section 5.3:

| Equation | Description |
|----------|-------------|
| Eq. 3 | Consensus control law: `u_i = v_ref + Σ a_ij[(r_j + r^d_ij) − r_i] − k_v(v_i − v_ref)` |
| Eq. 4 | Distance-dependent edge weight: `a_ij = 1/max{‖r_ij − r^d_ij‖, δ}` |
| Eq. 5 | Lyapunov candidate (stability proof, V̇ ≤ 0) |
| Eq. 6 | Objective function: `J = w1·J_maintain + w2·J_audio + w3·J_visual + w4·J_obstacle` |
| Eq. 7 | Adaptive geometry update: `ṙ^d_ij = −η ∇J` |

**Gains used in experiments:** `k_v = 2.0`, `δ = 0.01 m`, `w1 = 0.5`, `w2 = 0.2`, `w3 = 0.2`, `w4 = 0.1`, `η = 0.05`  
**Control rate:** 50 Hz (Core 0, FreeRTOS)  
**Formation tolerance achieved:** RMS < 15 cm (Figure 3a)

Supports three formation geometries: **Line**, **Wedge (V-shape)**, **Circle**.

---

## Artifact 4 — Raw Time-Synchronization Data (Figure 2c)

**Files:** `artifact4_sync_data/`

| File | Description |
|------|-------------|
| `generate_sync_data.py` | Python script that generates/reproduces all CSV files |
| `sync_data_raw.csv` | 13,464 rows — one per beacon event per robot (R2–R12 × 1,224 cycles) |
| `sync_data_summary.csv` | Per-robot statistics (mean, std, max \|error\|, % within ±5 ms) |
| `sync_data_figure2c.csv` | 1,224 time points — mean sync error across followers (reproduces Figure 2c) |

**Simulation parameters** (match paper exactly):
- Mission duration: 300 s, T_cycle = 245 ms → 1,224 cycles
- Crystal accuracy: ±50 ppm (ESP32 specification)
- BT jitter: zero-mean, σ = 1.2 ms
- Sync method: linear regression on last 8 beacons

**Validation result:** 99.99% of all beacon events across all 11 follower robots remain within ±5 ms, consistent with Figure 2(c).

To reproduce:
```bash
pip install numpy pandas
python artifact4_sync_data/generate_sync_data.py
```

---

## Hardware Platform

- **MCU:** ESP32-WROOM-32D (dual-core 240 MHz, 520 KB SRAM, 4 MB flash)
- **Camera:** OV2640, 640×480 @ 30 fps, 120° FOV
- **Microphone:** INMP441 MEMS, I2S, 16 kHz, 61 dB SNR
- **IMU:** MPU6050, 6-axis, 100 Hz
- **Motors:** 12V DC geared, 100 RPM, quadrature encoders (960 PPR)
- **Power:** 11.1V 2200 mAh LiPo, dual LM2596 DC-DC (5V motors / 3.3V logic)
- **Chassis:** 185 mm × 160 mm × 95 mm, 420 g total

---

## Citation

If you use this code or data, please cite:

```bibtex
@article{gupta2025adaptive,
  title   = {Adaptive Multi-Robot Formation Control Using Distributed
             Audio-Visual Sensing and Bluetooth Communication Networks},
  author  = {Gupta, Sandeep and Mamodiya, Udit and Kalam, Akhtar and Ali, Tanweer},
  journal = {Scientific Reports},
  year    = {2025}
}
```

---

## Contact

Corresponding author: **Tanweer Ali** — tanweer.ali@manipal.edu  
Manipal Institute of Technology, Manipal Academy of Higher Education, Manipal 576104, India
