#include "main.h"
#include "scurve_trajectory.h"
#include "robot_arm.h"
#include "kalman.h"
#include <string.h>

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;

RobotArm_t g_arm;

volatile float TUNING_Q_pos_noise    = 0.001f;
volatile float TUNING_Q_vel_noise    = 0.01f;
volatile float TUNING_Q_current      = 0.1f;
volatile float TUNING_Q_disturbance  = 10.0f;
volatile float TUNING_R_sensor_noise = 0.05f;

volatile float COMMAND_target_deg    = 0.0f;
volatile float COMMAND_direction     = 1.0f;
volatile uint8_t TRIGGER_execute_move = 0;

int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM7_Init();

    RobotArm_Init(&g_arm);
    HAL_TIM_Base_Start_IT(&htim7);

    while (1) {
        g_arm.kf.Q_diag[0] = TUNING_Q_pos_noise;
        g_arm.kf.Q_diag[1] = TUNING_Q_vel_noise;
        g_arm.kf.Q_diag[2] = TUNING_Q_current;
        g_arm.kf.Q_diag[3] = TUNING_Q_disturbance;
        g_arm.kf.R_sensor  = TUNING_R_sensor_noise;

        if (TRIGGER_execute_move == 1) {
            TRIGGER_execute_move = 0;
            RobotArm_Move(&g_arm, COMMAND_target_deg, COMMAND_direction);
        }
        
        HAL_Delay(10);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM7) {
        RobotArm_ControlTick(&g_arm);
    }
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {}
}