/**
 * @file    kalman.h
 * @brief   4-State Discrete Kalman Filter for DC Motor State Estimation
 * Designed for STM32G474RE | FRA233 Lab 4 Control Integration
 *
 * States:
 * x[0] = Position (theta)          [rad]
 * x[1] = Velocity (omega)          [rad/s]
 * x[2] = Armature Current (i)      [A]
 * x[3] = Disturbance Torque (tau_d)[N*m]
 *
 * Measurement:
 * y    = Position (theta)          [rad]
 *
 * Input:
 * u    = Terminal Voltage (V)      [V]
 */

#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>
#include <stdbool.h>

/* ─── KALMAN FILTER STRUCT ──────────────────────────────────────────────── */
typedef struct {
    float x[4];        /* State vector: [theta_rad, omega_rad_s, current_amp, torque_nm] */
    float P[4][4];     /* Error Covariance Matrix */
    float Q_diag[4];   /* Process Noise Covariance Diagonal Elements */
    float R_sensor;    /* Measurement Noise Covariance (Scalar) */
} KalmanFilter_t;

/* ─── PUBLIC API ────────────────────────────────────────────────────────── */

/**
 * @brief  Initializes the Kalman Filter with default states and covariances.
 * @param  kf  Pointer to the KalmanFilter_t structure.
 */
void KalmanFilter_Init(KalmanFilter_t *kf);

/**
 * @brief  Executes the Predict Step (A priori state and covariance propagation).
 * Call this at the beginning of your 1ms control loop cycle.
 * @param  kf       Pointer to the KalmanFilter_t structure.
 * @param  u_volt   Applied motor terminal voltage [Volts].
 */
void KalmanFilter_Predict(KalmanFilter_t *kf, float u_volt);

/**
 * @brief  Executes the Update Step (A posteriori measurement correction).
 * Call this immediately after reading the raw encoder position.
 * @param  kf           Pointer to the KalmanFilter_t structure.
 * @param  y_meas_deg   Raw measured encoder position in [Degrees].
 */
void KalmanFilter_Update(KalmanFilter_t *kf, float y_meas_deg);

/* ─── GETTERS (CONVERTED TO PRACTICAL UNITS) ────────────────────────────── */
float Kalman_GetPositionDeg(const KalmanFilter_t *kf);
float Kalman_GetVelocityDegS(const KalmanFilter_t *kf);
float Kalman_GetCurrentAmp(const KalmanFilter_t *kf);
float Kalman_GetDisturbanceNm(const KalmanFilter_t *kf);

#endif /* KALMAN_H */
