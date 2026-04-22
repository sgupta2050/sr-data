/**
 * formation_control.h
 * ============================================================
 * Distributed Consensus Formation Controller + Adaptive
 * Formation Optimizer — ESP32 Multi-Robot System
 *
 * Companion code for:
 *   "Adaptive Multi-Robot Formation Control Using Distributed
 *    Audio-Visual Sensing and Bluetooth Communication Networks"
 *   Scientific Reports, 2025.
 *
 * Repository: https://github.com/sgupta2050/sr-data
 *
 * Implements Equations 3–7 from the paper:
 *
 *  Control law (Eq. 3):
 *    u_i = v_ref + Σ_{j∈N_i} a_ij [(r_j + r^d_ij) − r_i] − k_v(v_i − v_ref)
 *
 *  Distance-dependent edge weight (Eq. 4):
 *    a_ij = 1 / max{ ||r_ij − r^d_ij||, δ }
 *
 *  Lyapunov candidate (Eq. 5) — proved V̇ ≤ 0 → asymptotic stability
 *
 *  Objective function (Eq. 6):
 *    J(r^d) = w1*J_maintain + w2*J_audio + w3*J_visual + w4*J_obstacle
 *
 *  Adaptive geometry update (Eq. 7):
 *    ṙ^d_ij = −η ∇_{r^d_ij} J(r^d)
 *
 * Control loop: 50 Hz (Core 0, FreeRTOS)
 * Formation tolerance: ±150 mm RMS (Section 5.3, Figure 3a)
 * ============================================================
 */

#pragma once
#include <Arduino.h>
#include <math.h>
#include "bt_spp.h"     // RobotState_t

// ── Configuration ─────────────────────────────────────────────
#define FC_N_ROBOTS       12
#define FC_CTRL_HZ        50          // formation control loop rate
#define FC_CTRL_PERIOD_MS (1000 / FC_CTRL_HZ)   // 20 ms

// Consensus controller gains
#define FC_KV             2.0f        // velocity damping gain k_v
#define FC_DELTA          0.01f       // zero-division guard δ (m)

// Adaptive formation weights (Eq. 6)
#define FC_W1             0.5f        // maintain nominal formation
#define FC_W2             0.2f        // acoustic objective
#define FC_W3             0.2f        // visual / line-of-sight
#define FC_W4             0.1f        // obstacle avoidance
#define FC_ETA            0.05f       // gradient descent rate η

// Formation geometry selection
typedef enum {
    FORM_LINE    = 0,
    FORM_WEDGE   = 1,
    FORM_CIRCLE  = 2,
} FormationType_t;

// 2-D vector
typedef struct { float x, y; } Vec2_t;

// Per-pair desired relative position (r^d_ij)
typedef struct {
    Vec2_t r_desired[FC_N_ROBOTS][FC_N_ROBOTS];
} FormationGeom_t;

// ── Public API ────────────────────────────────────────────────

/**
 * fc_init()
 * Initialise the formation controller.
 *
 * @param robot_id   This robot's ID (1-indexed)
 * @param n_robots   Total swarm size
 * @param type       Initial formation geometry
 * @param spacing_m  Inter-robot spacing in metres (e.g. 0.5)
 */
void fc_init(uint8_t robot_id, uint8_t n_robots,
             FormationType_t type, float spacing_m);

/**
 * fc_set_formation()
 * Change the desired formation geometry at runtime.
 * Triggers adaptive reconfiguration (Eq. 7).
 */
void fc_set_formation(FormationType_t type, float spacing_m);

/**
 * fc_update()
 * Run one 50 Hz control step.
 * Reads neighbour states from the shared buffer,
 * computes Eq. 3, returns motor velocity commands.
 *
 * @param self        Current robot's state (from odometry)
 * @param neighbours  Array of N_ROBOTS neighbour states
 *                    (from TDMA state buffer)
 * @param v_ref       Reference velocity from coordination layer
 * @param[out] u      Computed control velocity (x, y)
 */
void fc_update(const RobotState_t *self,
               const RobotState_t *neighbours,
               Vec2_t v_ref,
               Vec2_t *u);

/**
 * fc_adapt_audio()
 * Inject an acoustic source position estimate to update
 * J_audio gradient term (Eq. 6, w2 term).
 *
 * @param source_pos  Estimated source position in world frame
 */
void fc_adapt_audio(Vec2_t source_pos);

/**
 * fc_adapt_visual()
 * Inject a visual landmark position to update J_visual
 * (line-of-sight maintenance term, w3).
 */
void fc_adapt_visual(Vec2_t landmark_pos);

/**
 * fc_adapt_obstacle()
 * Inject an obstacle position to update J_obstacle (w4).
 * Called by ultrasonic sensor interrupt handler.
 */
void fc_adapt_obstacle(Vec2_t obstacle_pos, float radius_m);

/**
 * fc_get_lyapunov()
 * Returns current value of Lyapunov function V (Eq. 5).
 * Useful for stability monitoring and logging.
 */
float fc_get_lyapunov(const RobotState_t *neighbours);
