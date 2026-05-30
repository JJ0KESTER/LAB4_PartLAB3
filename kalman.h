/**
 * @file    kalman.h
 * @brief   4-State Discrete Kalman Filter with Exact Qd Matrix for STM32CubeIDE
 * Designed for FRA233 Lab 4 | Control Frequency: 1 kHz (Ts = 0.001s)
 */

#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>
#include <stdbool.h>

/* ─── KALMAN FILTER STRUCT ──────────────────────────────────────────────── */
typedef struct {
    float x[4];        /* State vector: [theta_rad, omega_rad_s, current_amp, torque_nm] */
    float P[4][4];     /* Error Covariance Matrix */
} KalmanFilter_t;

/* ─── PUBLIC API ────────────────────────────────────────────────────────── */

/**
 * @brief  Initializes the Kalman Filter with initial states and error covariance.
 * @param  kf  Pointer to the KalmanFilter_t structure.
 */
void KalmanFilter_Init(KalmanFilter_t *kf);

/**
 * @brief  Predict Step (A priori state and covariance propagation)
 * Runs at the beginning of the 1ms loop.
 * @param  kf       Pointer to the KalmanFilter_t structure.
 * @param  u_volt   Applied motor terminal voltage [Volts].
 */
void KalmanFilter_Predict(KalmanFilter_t *kf, float u_volt);

/**
 * @brief  Update Step (A posteriori measurement correction)
 * Runs after reading the encoder.
 * @param  kf           Pointer to the KalmanFilter_t structure.
 * @param  y_meas_deg   Raw measured encoder position in [Degrees].
 */
void KalmanFilter_Update(KalmanFilter_t *kf, float y_meas_deg);

/* ─── GETTERS (CONVERTED TO PRACTICAL UNITS FOR PID / LOGGING) ──────────── */
float Kalman_GetPositionDeg(const KalmanFilter_t *kf);
float Kalman_GetVelocityDegS(const KalmanFilter_t *kf);
float Kalman_GetCurrentAmp(const KalmanFilter_t *kf);
float Kalman_GetDisturbanceNm(const KalmanFilter_t *kf);

#endif /* KALMAN_H */