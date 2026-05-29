#ifndef SCURVE_TRAJECTORY_H
#define SCURVE_TRAJECTORY_H

#include <stdint.h>
#include <stdbool.h>

#define TRAJ_SAMPLE_RATE_HZ     1000.0f
#define TRAJ_DT                 (1.0f / TRAJ_SAMPLE_RATE_HZ)

#define WAYPOINT_COUNT          6
extern const float WAYPOINTS_DEG[WAYPOINT_COUNT];

#define SCURVE_V_MAX_DEG_S      1000.0f
#define SCURVE_A_MAX_DEG_S2     1000.0f
#define SCURVE_J_MAX_DEG_S3     300.0f

typedef enum {
    TRAJ_IDLE = 0,
    TRAJ_PHASE1,
    TRAJ_PHASE2,
    TRAJ_PHASE3,
    TRAJ_PHASE4,
    TRAJ_PHASE5,
    TRAJ_PHASE6,
    TRAJ_PHASE7
} TrajPhase_t;

typedef struct {
    float position_deg;
    float velocity_deg_s;
    float accel_deg_s2;
} TrajOutput_t;

typedef struct {
    float p;
    float v;
} PhaseIC_t;

typedef struct {
    float q_start;
    float q_end;
    float direction;
    float total_disp;

    float v_max_eff;
    float a_max_eff;
    float j_max;

    float t1, t2, t3, t4, t5, t6, t7;
    PhaseIC_t ic[7];

    TrajPhase_t phase;
    float       t_phase;

    TrajOutput_t out;

    uint8_t wp_index;
    bool    active;
} SCurveTraj_t;

void SCurve_Init(SCurveTraj_t *traj);
void SCurve_SetSegment(SCurveTraj_t *traj, float q_start, float q_end);
TrajOutput_t SCurve_Update(SCurveTraj_t *traj);
bool SCurve_IsDone(const SCurveTraj_t *traj);

#endif