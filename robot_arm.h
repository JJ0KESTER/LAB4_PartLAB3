#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <stdint.h>
#include <stdbool.h>
#include "scurve_trajectory.h"
#include "kalman.h"

#define ARM_PULSES_PER_REV      8192
#define ARM_TIM_PWM_PERIOD      999
#define ARM_PWM_MAX             950

typedef struct {
    float kp_pos;
    float kp_vel;
    float ki_vel;
    float kv_ff;

    float pos_error_integral;
    float vel_error_integral;
    float prev_pos_error;
    float prev_vel_error;
    
    float u_volt_cmd;

    SCurveTraj_t    traj;
    KalmanFilter_t  kf;

    float           pos_deg;
    float           vel_deg_s;
    float           current_amp;
    float           disturbance_nm;
    
    float           raw_enc_deg;

    float           ref_pos_deg;
    float           ref_vel_deg_s;

    bool            running;
    bool            done;
} RobotArm_t;

void RobotArm_Init(RobotArm_t *arm);
void RobotArm_Move(RobotArm_t *arm, float degrees, float direction);
bool RobotArm_IsDone(const RobotArm_t *arm);
void RobotArm_Stop(RobotArm_t *arm);
void RobotArm_ControlTick(RobotArm_t *arm);

#endif