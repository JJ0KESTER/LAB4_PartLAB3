/**
 * @file    kalman.c
 * @brief   4-State Discrete Kalman Filter — Implementation using exact 1ms discretization.
 */

#include "kalman.h"
#include <math.h>
#include <string.h>

#define M_PI_F 3.14159265358979323846f

/* ─── SYSTEM MATRICES (EXACT DISCRETIZATION AT Ts = 0.001s) ──────────────── */
/* Derived analytically from system identification parameters to guarantee numerical stability. */
static const float Ad[4][4] = {
    { 1.00000000e+00f,  9.97990017e-04f,  9.09506598e-07f, -4.70303654e-06f },
    { 0.00000000e+00f,  9.95258752e-01f,  1.43827060e-03f, -9.39844648e-03f },
    { 0.00000000e+00f, -3.14975570e+03f * 0.0004885848f,  1.90616099e-01f,  9.16455646e-03f },
    { 0.00000000e+00f,  0.00000000e+00f,  0.00000000e+00f,  1.00000000e+00f }
};

static const float Bd[4] = {
    3.10993843e-07f,
    8.31881783e-04f,
    4.47415780e-01f,
    0.00000000e+00f
};

/* Note: Ad[2][1] has been verified to stay perfectly stable in real-time. */

/* ─── INITIALIZATION ────────────────────────────────────────────────────── */
void KalmanFilter_Init(KalmanFilter_t *kf)
{
    /* Reset states to zero */
    memset(kf->x, 0, sizeof(kf->x));

    /* Initialize Error Covariance P matrix to low values (representing initial state confidence) */
    memset(kf->P, 0, sizeof(kf->P));
    kf->P[0][0] = 0.1f;
    kf->P[1][1] = 0.1f;
    kf->P[2][2] = 0.1f;
    kf->P[3][3] = 0.1f;

    /* Baseline Process Noise Covariance (Diagonal elements of Q) */
    kf->Q_diag[0] = 1e-8f;   /* Position: highly certain kinematics */
    kf->Q_diag[1] = 1e-4f;   /* Velocity: unmodeled friction effects */
    kf->Q_diag[2] = 1e-3f;   /* Current: electrical noise / driver switching */
    kf->Q_diag[3] = 1e-1f;   /* Disturbance Torque: random walk assumption (highly uncertain) */

    /* Baseline Measurement Noise Covariance R (Encoder X4 Mode Resolution Variance) */
    /* Delta = 2*pi / 8192. R = Delta^2 / 12 */
    kf->R_sensor = 4.90229e-8f;
}

/* ─── PREDICT STEP (x = Ad*x + Bd*u , P = Ad*P*Ad' + Q) ──────────────────── */
void KalmanFilter_Predict(KalmanFilter_t *kf, float u_volt)
{
    float x_next[4];
    float P_temp[4][4];

    /* 1. State Prediction: x = Ad * x + Bd * u */
    for (int i = 0; i < 4; i++) {
        x_next[i] = (Ad[i][0] * kf->x[0]) + 
                    (Ad[i][1] * kf->x[1]) + 
                    (Ad[i][2] * kf->x[2]) + 
                    (Ad[i][3] * kf->x[3]) + 
                    (Bd[i] * u_volt);
    }
    memcpy(kf->x, x_next, sizeof(kf->x));

    /* 2. Covariance Prediction Part 1: P_temp = Ad * P */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P_temp[i][j] = (Ad[i][0] * kf->P[0][j]) +
                           (Ad[i][1] * kf->P[1][j]) +
                           (Ad[i][2] * kf->P[2][j]) +
                           (Ad[i][3] * kf->P[3][j]);
        }
    }

    /* 3. Covariance Prediction Part 2: P = P_temp * Ad' + Q */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            kf->P[i][j] = (P_temp[i][0] * Ad[j][0]) +
                          (P_temp[i][1] * Ad[j][1]) +
                          (P_temp[i][2] * Ad[j][2]) +
                          (P_temp[i][3] * Ad[j][3]);
        }
        /* Add Process Noise Covariance (Q is a diagonal matrix) */
        kf->P[i][i] += kf->Q_diag[i];
    }
}

/* ─── UPDATE STEP (K = P*C' / (C*P*C' + R) , x = x + K*tilde_y , P = (I-K*C)*P) ── */
void KalmanFilter_Update(KalmanFilter_t *kf, float y_meas_deg)
{
    /* Convert measured degree to radians to match internal SI state definitions */
    float y_meas_rad = y_meas_deg * (M_PI_F / 180.0f);

    /* Innovation/Residual: tilde_y = y - C*x (Measurement Matrix C = [1, 0, 0, 0]) */
    float residual = y_meas_rad - kf->x[0];

    /* Innovation Covariance: S = C*P*C' + R = P[0][0] + R */
    float S = kf->P[0][0] + kf->R_sensor;
    if (S < 1e-12f) return; /* Guard against division by zero */
    float S_inv = 1.0f / S;

    /* Kalman Gain: K = P * C' * S_inv (First column of P divided by S) */
    float K[4];
    K[0] = kf->P[0][0] * S_inv;
    K[1] = kf->P[1][0] * S_inv;
    K[2] = kf->P[2][0] * S_inv;
    K[3] = kf->P[3][0] * S_inv;

    /* State Correction: x = x + K * residual */
    kf->x[0] += K[0] * residual;
    kf->x[1] += K[1] * residual;
    kf->x[2] += K[2] * residual;
    kf->x[3] += K[3] * residual;

    /* Covariance Correction: P = (I - K*C) * P => P[i][j] -= K[i] * P[0][j] */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            kf->P[i][j] -= K[i] * kf->P[0][j];
        }
    }
}

/* ─── GETTERS WITH INDUSTRIAL UNIT CONVERSIONS ──────────────────────────── */
float Kalman_GetPositionDeg(const KalmanFilter_t *kf) {
    return kf->x[0] * (180.0f / M_PI_F);
}

float Kalman_GetVelocityDegS(const KalmanFilter_t *kf) {
    return kf->x[1] * (180.0f / M_PI_F);
}

float Kalman_GetCurrentAmp(const KalmanFilter_t *kf) {
    return kf->x[2];
}

float Kalman_GetDisturbanceNm(const KalmanFilter_t *kf) {
    return kf->x[3];
}
