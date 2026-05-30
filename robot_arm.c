#include "kalman.h"

// 1. ประกาศตัวแปรโครงสร้างไว้ที่ส่วนหัวไฟล์ robot_arm.c
KalmanFilter_t kf_motor;

// 2. เรียกฟังก์ชันนี้ในฟังก์ชัน RobotArm_Init()
KalmanFilter_Init(&kf_motor);

// 3. ในลูปควบคุมย่อย (TIM7 ISR ทุกๆ 1 ms)
void RobotArm_ControlLoop(RobotArm_t *arm)
{
    float dt = 0.001f;
    
    /* ── 1. Read hardware encoder ── */
    arm->pos_deg = COUNTS_TO_DEG((float)__HAL_TIM_GET_COUNTER(&htim2));

    /* ── 2. KALMAN FILTER EXECUTIONS ── */
    // แปลงสัญญาณ PWM สั่งการรอบที่แล้วกลับเป็นแรงดันไฟฟ้าจริง (สมมติแบตเตอรี่ 24V)
    float u_volt = (arm->prev_pwm_cmd / 999.0f) * 24.0f; 
    
    // รันกระบวนการ Kalman Filter
    KalmanFilter_Predict(&kf_motor, u_volt);
    KalmanFilter_Update(&kf_motor, arm->pos_deg);

    // แทนที่การหาความเร็วแบบเก่า (Finite Difference) ด้วยความเร็วที่สกัดจาก Kalman Filter!
    arm->vel_deg_s = Kalman_GetVelocityDegS(&kf_motor); 
    
    /* ── 3. Run S-Curve & Cascade PID ต่อตามปกติ ── */
    // ... โค้ดควบคุมเดิมของคุณ ...
    
    // บันทึกค่า PWM รอบนี้ไว้ใช้ทำนายในรอบถัดไป
    arm->prev_pwm_cmd = pwm_cmd; 
}