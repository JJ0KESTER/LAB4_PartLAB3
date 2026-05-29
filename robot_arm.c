#include "robot_arm.h"
#include "main.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

void RobotArm_Init(RobotArm_t *arm) {
    arm->kp_pos = 2.5f;
    arm->kp_vel = 0.15f;
    arm->ki_vel = 0.01f;
    arm->kv_ff  = 0.005f;

    arm->pos_error_integral = 0.0f;
    arm->vel_error_integral = 0.0f;
    arm->prev_pos_error = 0.0f;
    arm->prev_vel_error = 0.0f;
    arm->u_volt_cmd = 0.0f;

    arm->pos_deg = 0.0f;
    arm->vel_deg_s = 0.0f;
    arm->current_amp = 0.0f;
    arm->disturbance_nm = 0.0f;
    arm->raw_enc_deg = 0.0f;

    arm->ref_pos_deg = 0.0f;
    arm->ref_vel_deg_s = 0.0f;
    arm->running = false;
    arm->done = true;

    KalmanFilter_Init(&arm->kf);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}

void RobotArm_Move(RobotArm_t *arm, float degrees, float direction) {
    if (degrees <= 0.0f) return;
    
    float q_start = arm->pos_deg;
    float q_end = q_start + (degrees * direction);

    SCurve_SetSegment(&arm->traj, q_start, q_end);
    
    arm->running = true;
    arm->done = false;
}

bool RobotArm_IsDone(const RobotArm_t *arm) {
    return arm->done;
}

void RobotArm_Stop(RobotArm_t *arm) {
    arm->running = false;
    arm->done = true;
    arm->ref_pos_deg = arm->pos_deg;
    arm->ref_vel_deg_s = 0.0f;
    arm->pos_error_integral = 0.0f;
    arm->vel_error_integral = 0.0f;
    
    arm->u_volt_cmd = 0.0f;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}

void RobotArm_ControlTick(RobotArm_t *arm) {
    KalmanFilter_Predict(&arm->kf, arm->u_volt_cmd);

    int16_t enc_raw = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    arm->raw_enc_deg = ((float)enc_raw / (float)ARM_PULSES_PER_REV) * 360.0f;

    KalmanFilter_Update(&arm->kf, arm->raw_enc_deg);

    arm->pos_deg = Kalman_GetPositionDeg(&arm->kf);
    arm->vel_deg_s = Kalman_GetVelocityDegS(&arm->kf);
    arm->current_amp = Kalman_GetCurrentAmp(&arm->kf);
    arm->disturbance_nm = Kalman_GetDisturbanceNm(&arm->kf);

    if (arm->running) {
        TrajOutput_t ref = SCurve_Update(&arm->traj);
        arm->ref_pos_deg = ref.position_deg;
        arm->ref_vel_deg_s = ref.velocity_deg_s;

        if (SCurve_IsDone(&arm->traj) && fabsf(arm->vel_deg_s) < 1.0f) {
            arm->running = false;
            arm->done = true;
        }
    }

    float pos_error = arm->ref_pos_deg - arm->pos_deg;
    float vel_ref_from_pos = pos_error * arm->kp_pos;
    float total_vel_ref = vel_ref_from_pos + arm->ref_vel_deg_s;

    float vel_error = total_vel_ref - arm->vel_deg_s;
    arm->vel_error_integral += vel_error * 0.001f;

    if (arm->vel_error_integral > 24.0f)  arm->vel_error_integral = 24.0f;
    if (arm->vel_error_integral < -24.0f) arm->vel_error_integral = -24.0f;

    float p_term = vel_error * arm->kp_vel;
    float i_term = arm->vel_error_integral * arm->ki_vel;
    float ff_term = total_vel_ref * arm->kv_ff;

    arm->u_volt_cmd = p_term + i_term + ff_term;

    if (arm->u_volt_cmd >= 0.0f) {
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
    }

    float volt_magnitude = fabsf(arm->u_volt_cmd);
    uint32_t pwm_duty = (uint32_t)((volt_magnitude / 24.0f) * (float)ARM_TIM_PWM_PERIOD);

    if (pwm_duty > ARM_PWM_MAX) pwm_duty = ARM_PWM_MAX;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_duty);
}