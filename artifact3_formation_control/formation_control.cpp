/**
 * formation_control.cpp
 * ============================================================
 * Formation Controller Implementation — Equations 3–7
 * Repository: https://github.com/sgupta2050/sr-data
 * ============================================================
 */

#include "formation_control.h"
#include <string.h>
#include <math.h>

// ── Internal state ────────────────────────────────────────────
static uint8_t         g_id;
static uint8_t         g_n;
static FormationGeom_t g_geom;          // current desired r^d_ij
static FormationGeom_t g_geom_nominal;  // nominal (undisturbed) geometry

// Adaptive perturbation accumulators (for Eq. 7)
static Vec2_t g_audio_source   = {0, 0};
static Vec2_t g_visual_target  = {0, 0};
static Vec2_t g_obstacle_pos   = {0, 0};
static float  g_obstacle_r     = 0.0f;
static bool   g_audio_active   = false;
static bool   g_visual_active  = false;
static bool   g_obstacle_active= false;

// ── Geometry generation ───────────────────────────────────────
/**
 * generate_nominal_geom()
 * Fills g_geom_nominal.r_desired[i][j] with the desired
 * relative position of robot j as seen from robot i,
 * for the given formation type and inter-robot spacing.
 *
 * Line:   robots in a row along x-axis
 * Wedge:  V-shape, leader at apex (robot 1)
 * Circle: uniform angular spacing on a circle of radius
 *         spacing / (2 * sin(π/N))
 */
static void generate_nominal_geom(FormationType_t type,
                                   float spacing_m) {
    // Compute absolute positions in formation frame first
    Vec2_t abs_pos[FC_N_ROBOTS] = {{0}};

    if (type == FORM_LINE) {
        for (int i = 0; i < g_n; i++) {
            abs_pos[i].x = i * spacing_m;
            abs_pos[i].y = 0.0f;
        }
    } else if (type == FORM_WEDGE) {
        // Leader at origin; followers in V behind
        abs_pos[0] = {0.0f, 0.0f};
        for (int i = 1; i < g_n; i++) {
            int side  = (i % 2 == 1) ? 1 : -1;
            int depth = (i + 1) / 2;
            abs_pos[i].x = -(float)depth * spacing_m * cosf(M_PI / 6.0f);
            abs_pos[i].y =  (float)side  * depth * spacing_m * sinf(M_PI / 6.0f);
        }
    } else {  // FORM_CIRCLE
        float radius = (g_n > 1)
                       ? spacing_m / (2.0f * sinf(M_PI / g_n))
                       : spacing_m;
        for (int i = 0; i < g_n; i++) {
            float angle = 2.0f * M_PI * i / g_n;
            abs_pos[i].x = radius * cosf(angle);
            abs_pos[i].y = radius * sinf(angle);
        }
    }

    // r^d_ij = abs_pos[j] - abs_pos[i]
    for (int i = 0; i < g_n; i++)
        for (int j = 0; j < g_n; j++) {
            g_geom_nominal.r_desired[i][j].x =
                abs_pos[j].x - abs_pos[i].x;
            g_geom_nominal.r_desired[i][j].y =
                abs_pos[j].y - abs_pos[i].y;
        }
    memcpy(&g_geom, &g_geom_nominal, sizeof(FormationGeom_t));
}

// ── Public: init ──────────────────────────────────────────────
void fc_init(uint8_t robot_id, uint8_t n_robots,
             FormationType_t type, float spacing_m) {
    g_id = robot_id;
    g_n  = (n_robots <= FC_N_ROBOTS) ? n_robots : FC_N_ROBOTS;
    generate_nominal_geom(type, spacing_m);
}

void fc_set_formation(FormationType_t type, float spacing_m) {
    generate_nominal_geom(type, spacing_m);
}

// ── Edge weight (Eq. 4) ──────────────────────────────────────
static float edge_weight(const Vec2_t *r_ij, const Vec2_t *r_d_ij) {
    float dx  = r_ij->x - r_d_ij->x;
    float dy  = r_ij->y - r_d_ij->y;
    float err = sqrtf(dx*dx + dy*dy);
    return 1.0f / fmaxf(err, FC_DELTA);   // Eq. 4
}

// ── Control law (Eq. 3) ───────────────────────────────────────
void fc_update(const RobotState_t *self,
               const RobotState_t *neighbours,
               Vec2_t v_ref,
               Vec2_t *u) {

    int i = g_id - 1;  // 0-indexed

    // Consensus term: Σ_{j∈N_i} a_ij [(r_j + r^d_ij) − r_i]
    Vec2_t consensus = {0.0f, 0.0f};
    for (int j = 0; j < g_n; j++) {
        if (j == i) continue;
        if (neighbours[j].robot_id == 0) continue;  // not yet received

        // r_ij = r_j - r_i (actual relative position)
        Vec2_t r_ij = {
            neighbours[j].pos_x - self->pos_x,
            neighbours[j].pos_y - self->pos_y
        };
        Vec2_t *r_d_ij = &g_geom.r_desired[i][j];

        float a_ij = edge_weight(&r_ij, r_d_ij);

        // Eq. 3: attraction toward target position (r_j + r^d_ij) − r_i
        // When robot i is displaced behind, this term is positive → pulls forward
        // When in perfect formation, term = r^d_ij + r_ij → large, balanced by k_v damping at steady-state
        float target_x = neighbours[j].pos_x + r_d_ij->x - self->pos_x;
        float target_y = neighbours[j].pos_y + r_d_ij->y - self->pos_y;

        consensus.x += a_ij * target_x;
        consensus.y += a_ij * target_y;
    }

    // Velocity damping: −k_v (v_i − v_ref)
    float damp_x = -FC_KV * (self->vel_x - v_ref.x);
    float damp_y = -FC_KV * (self->vel_y - v_ref.y);

    // Full control law (Eq. 3)
    u->x = v_ref.x + consensus.x + damp_x;
    u->y = v_ref.y + consensus.y + damp_y;

    // ── Adaptive geometry update (Eq. 7) ────────────────────
    // Gradient descent on J: ṙ^d_ij = −η ∇ J
    // Applied once per control step (integrator in the geometry)
    for (int j = 0; j < g_n; j++) {
        if (j == i) continue;
        Vec2_t *rd = &g_geom.r_desired[i][j];
        Vec2_t *rd_nom = &g_geom_nominal.r_desired[i][j];

        // J_maintain gradient: penalise deviation from nominal
        float gx = FC_W1 * (rd->x - rd_nom->x);
        float gy = FC_W1 * (rd->y - rd_nom->y);

        // J_audio: steer formation toward acoustic source
        if (g_audio_active) {
            float dx = self->pos_x + rd->x - g_audio_source.x;
            float dy = self->pos_y + rd->y - g_audio_source.y;
            gx += FC_W2 * dx;
            gy += FC_W2 * dy;
        }

        // J_visual: maintain line-of-sight to visual landmark
        if (g_visual_active) {
            float dx = self->pos_x + rd->x - g_visual_target.x;
            float dy = self->pos_y + rd->y - g_visual_target.y;
            gx += FC_W3 * dx;
            gy += FC_W3 * dy;
        }

        // J_obstacle: repulsion
        if (g_obstacle_active) {
            float dx  = self->pos_x + rd->x - g_obstacle_pos.x;
            float dy  = self->pos_y + rd->y - g_obstacle_pos.y;
            float dist = sqrtf(dx*dx + dy*dy);
            if (dist < g_obstacle_r + 0.30f) {  // 30 cm margin
                float rep = FC_W4 / fmaxf(dist, 0.05f);
                gx -= rep * dx;
                gy -= rep * dy;
            }
        }

        // Gradient step (Eq. 7)
        rd->x -= FC_ETA * gx;
        rd->y -= FC_ETA * gy;
    }
}

// ── Adaptation injectors ──────────────────────────────────────
void fc_adapt_audio(Vec2_t source_pos) {
    g_audio_source = source_pos;
    g_audio_active = true;
}

void fc_adapt_visual(Vec2_t landmark_pos) {
    g_visual_target = landmark_pos;
    g_visual_active = true;
}

void fc_adapt_obstacle(Vec2_t obstacle_pos, float radius_m) {
    g_obstacle_pos    = obstacle_pos;
    g_obstacle_r      = radius_m;
    g_obstacle_active = true;
}

// ── Lyapunov monitor (Eq. 5) ──────────────────────────────────
float fc_get_lyapunov(const RobotState_t *neighbours) {
    int   i   = g_id - 1;
    float V   = 0.0f;
    const RobotState_t *self = &neighbours[i];

    for (int j = 0; j < g_n; j++) {
        if (j == i) continue;
        if (neighbours[j].robot_id == 0) continue;

        Vec2_t r_ij = {
            neighbours[j].pos_x - self->pos_x,
            neighbours[j].pos_y - self->pos_y
        };
        Vec2_t *r_d = &g_geom.r_desired[i][j];
        float a_ij  = edge_weight(&r_ij, r_d);
        float ex    = r_ij.x - r_d->x;
        float ey    = r_ij.y - r_d->y;
        V += 0.5f * a_ij * (ex*ex + ey*ey);     // position term

        float dv_x = self->vel_x;
        float dv_y = self->vel_y;
        V += (FC_KV / 2.0f) * (dv_x*dv_x + dv_y*dv_y); // velocity term
    }
    return V;
}
