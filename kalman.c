<<<<<<< Updated upstream:kalman.c
/**
 * @file    kalman.c
 * @brief   4-State Discrete Kalman Filter Implementation with Exact Discretization.
 */

#include "kalman.h"
#include <string.h>

#define M_PI_F 3.14159265358979323846f

/* ─── SYSTEM MATRICES (EXACT DISCRETIZATION AT Ts = 0.001s) ──────────────── */

/* 1. State Transition Matrix (Ad) */
static const float Ad[4][4] = {
    { 1.00000000e+00f,  9.97990017e-04f,  9.09506598e-07f, -4.70303654e-06f },
    { 0.00000000e+00f,  9.95258752e-01f,  1.43827060e-03f, -9.39844648e-03f },
    { 0.00000000e+00f, -1.53892100e+00f,  1.90616099e-01f,  9.16455646e-03f },
    { 0.00000000e+00f,  0.00000000e+00f,  0.00000000e+00f,  1.00000000e+00f }
};

/* 2. Input Matrix (Bd) */
static const float Bd[4] = {
    3.10993843e-07f,
    8.31881783e-04f,
    4.47415780e-01f,
    0.00000000e+00f
};

/* 3. Exact Discrete Process Noise Covariance Matrix (Qd) */
/* คำนวณผ่านวิธี Van Loan's Method จาก Continuous Noise Qc ของกลุ่มคุณ */
static const float Qd[4][4] = {
    {  1.000034e-09f,  5.090554e-11f, -6.167558e-11f, -1.568259e-10f },
    {  5.090554e-11f,  1.025476e-07f, -9.906813e-08f, -4.703037e-07f },
    { -6.167558e-11f, -9.906813e-08f,  4.071285e-07f,  3.426113e-07f },
    { -1.568259e-10f, -4.703037e-07f,  3.426113e-07f,  1.000000e-04f }
};

/* 4. Measurement Noise Covariance (R) - จากความละเอียด Encoder โหมด X4 */
static const float R_sensor = 4.90229e-8f;


/* ─── INITIALIZATION ────────────────────────────────────────────────────── */
void KalmanFilter_Init(KalmanFilter_t *kf)
{
    /* ตั้งค่าสถานะเริ่มต้น (Initial States) เป็น 0 ทั้งหมด */
    memset(kf->x, 0, sizeof(kf->x));

    /* ตั้งค่า Error Covariance Matrix (P) เริ่มต้นเป็น Diagonal Matrix */
    memset(kf->P, 0, sizeof(kf->P));
    kf->P[0][0] = 0.1f;
    kf->P[1][1] = 0.1f;
    kf->P[2][2] = 0.1f;
    kf->P[3][3] = 1.0f; /* ให้ความไม่แน่นอนของ Disturbance สูงกว่าเล็กน้อยเพื่อให้ลู่เข้าไว */
}


/* ─── PREDICT STEP (x = Ad*x + Bd*u , P = Ad*P*Ad' + Qd) ──────────────────── */
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

    /* 3. Covariance Prediction Part 2: P = P_temp * Ad' + Qd */
    /* เปลี่ยนจากการบวกแค่แนวทแยง เป็นการบวกเมทริกซ์ Qd เต็มรูปแบบ (Exact) */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            kf->P[i][j] = (P_temp[i][0] * Ad[j][0]) +
                          (P_temp[i][1] * Ad[j][1]) +
                          (P_temp[i][2] * Ad[j][2]) +
                          (P_temp[i][3] * Ad[j][3]) + 
                          Qd[i][j];
        }
    }
}


/* ─── UPDATE STEP (K = P*C' / (C*P*C' + R) , x = x + K*residual , P = (I-K*C)*P) ── */
void KalmanFilter_Update(KalmanFilter_t *kf, float y_meas_deg)
{
    /* แปลงค่ามุมจากองศา (เซนเซอร์) เป็นเรเดียน ให้สอดคล้องกับ State-Space Model */
    float y_meas_rad = y_meas_deg * (M_PI_F / 180.0f);

    /* Innovation Residual: residual = y - C*x  (เนื่องจาก C = [1, 0, 0, 0]) */
    float residual = y_meas_rad - kf->x[0];

    /* Innovation Covariance: S = C*P*C' + R = P[0][0] + R */
    float S = kf->P[0][0] + R_sensor;
    if (S < 1e-12f) return; /* ตัวป้องกันการหารด้วยศูนย์ (Guard Clause) */
    float S_inv = 1.0f / S;

    /* Kalman Gain: K = P * C' * S_inv (ลดรูปพีชคณิตเหลือหลักแรกของเมทริกซ์ P คูณด้วย S_inv) */
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

    /* Covariance Correction: P = (I - K*C) * P (ลดรูปเพื่อประหยัดเวลาประมวลผลสูงสุด) */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            kf->P[i][j] -= K[i] * kf->P[0][j];
        }
    }
}


/* ─── GETTERS WITH UNIT CONVERSIONS ─────────────────────────────────────── */
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
=======
/**
 * @file    kalman.c
 * @brief   4-State Discrete Kalman Filter Implementation with Exact Discretization.
 */

#include "kalman.h"
#include <string.h>

#define M_PI_F 3.14159265358979323846f

/* ─── SYSTEM MATRICES (EXACT DISCRETIZATION AT Ts = 0.001s) ──────────────── */

/* 1. State Transition Matrix (Ad) */
static const float Ad[4][4] = {
    { 1.00000000e+00f,  9.97990017e-04f,  9.09506598e-07f, -4.70303654e-06f },
    { 0.00000000e+00f,  9.95258752e-01f,  1.43827060e-03f, -9.39844648e-03f },
    { 0.00000000e+00f, -1.53892100e+00f,  1.90616099e-01f,  9.16455646e-03f },
    { 0.00000000e+00f,  0.00000000e+00f,  0.00000000e+00f,  1.00000000e+00f }
};

/* 2. Input Matrix (Bd) */
static const float Bd[4] = {
    3.10993843e-07f,
    8.31881783e-04f,
    4.47415780e-01f,
    0.00000000e+00f
};

/* 3. Exact Discrete Process Noise Covariance Matrix (Qd) */
/* คำนวณผ่านวิธี Van Loan's Method จาก Continuous Noise Qc ของกลุ่มคุณ */
/* 3. Exact Discrete Process Noise Covariance Matrix (Qd) */

static const float Qd[4][4] = {
    {  1.000034e-06f,  5.090554e-08f, -6.167558e-08f, -1.568259e-07f },
    {  5.090554e-08f,  6.000000e-04f, -9.906813e-05f, -4.703037e-04f }, // [1][1] เพิ่มอีก 4e-4→6e-4
    { -6.167558e-08f, -9.906813e-05f,  4.071285e-04f,  3.426113e-04f },
    { -1.568259e-07f, -4.703037e-04f,  3.426113e-04f,  1.000000e-02f }
};

/* 4. Measurement Noise Covariance (R) - จากความละเอียด Encoder โหมด X4 */
static const float R_sensor = 2.0e-3f;


/* ─── INITIALIZATION ────────────────────────────────────────────────────── */
void KalmanFilter_Init(KalmanFilter_t *kf)
{
    /* ตั้งค่าสถานะเริ่มต้น (Initial States) เป็น 0 ทั้งหมด */
    memset(kf->x, 0, sizeof(kf->x));

    /* ตั้งค่า Error Covariance Matrix (P) เริ่มต้นเป็น Diagonal Matrix */
    memset(kf->P, 0, sizeof(kf->P));
    kf->P[0][0] = 0.1f;
    kf->P[1][1] = 0.1f;
    kf->P[2][2] = 0.1f;
    kf->P[3][3] = 1.0f; /* ให้ความไม่แน่นอนของ Disturbance สูงกว่าเล็กน้อยเพื่อให้ลู่เข้าไว */
}


/* ─── PREDICT STEP (x = Ad*x + Bd*u , P = Ad*P*Ad' + Qd) ──────────────────── */
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

    /* 3. Covariance Prediction Part 2: P = P_temp * Ad' + Qd */
    /* เปลี่ยนจากการบวกแค่แนวทแยง เป็นการบวกเมทริกซ์ Qd เต็มรูปแบบ (Exact) */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            kf->P[i][j] = (P_temp[i][0] * Ad[j][0]) +
                          (P_temp[i][1] * Ad[j][1]) +
                          (P_temp[i][2] * Ad[j][2]) +
                          (P_temp[i][3] * Ad[j][3]) + 
                          Qd[i][j];
        }
    }
}


/* ─── UPDATE STEP (K = P*C' / (C*P*C' + R) , x = x + K*residual , P = (I-K*C)*P) ── */
void KalmanFilter_Update(KalmanFilter_t *kf, float y_meas_deg)
{
    /* แปลงค่ามุมจากองศา (เซนเซอร์) เป็นเรเดียน ให้สอดคล้องกับ State-Space Model */
    float y_meas_rad = y_meas_deg * (M_PI_F / 180.0f);

    /* Innovation Residual: residual = y - C*x  (เนื่องจาก C = [1, 0, 0, 0]) */
    float residual = y_meas_rad - kf->x[0];

    /* Innovation Covariance: S = C*P*C' + R = P[0][0] + R */
    float S = kf->P[0][0] + R_sensor;
    if (S < 1e-12f) return; /* ตัวป้องกันการหารด้วยศูนย์ (Guard Clause) */
    float S_inv = 1.0f / S;

    /* Kalman Gain: K = P * C' * S_inv (ลดรูปพีชคณิตเหลือหลักแรกของเมทริกซ์ P คูณด้วย S_inv) */
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

    /* Covariance Correction: P = (I - K*C) * P (ลดรูปเพื่อประหยัดเวลาประมวลผลสูงสุด) */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            kf->P[i][j] -= K[i] * kf->P[0][j];
        }
    }
}


/* ─── GETTERS WITH UNIT CONVERSIONS ─────────────────────────────────────── */
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
>>>>>>> Stashed changes:Core/Src/kalman.c
}