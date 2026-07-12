#include "altitude.h"
#include <math.h>

void altitude_init(kalman_alt_t *kf, float initial_alt) {
kf->z = initial_alt;
kf->v = 0.0f;

// Initialize Error Covariance with confident diagonals
kf->P[0][0] = 1.0f; kf->P[0][1] = 0.0f;
kf->P[1][0] = 0.0f; kf->P[1][1] = 1.0f;

// Default Tuning Constants (Adjust based on sensor noise)
kf->Q_pos = 0.01f;   // How much we trust position kinematic updates
kf->Q_vel = 0.1f;    // How much we trust velocity kinematic updates
kf->R_baro = 0.25f;  // Barometer measurement variance (higher = trust baro less)


}

float baro_to_altitude(float pressure_pa, float temp_c) {
// Sea level standard pressure in Pascals (101325 Pa)
const float P0 = 101325.0f;

// Standard Hypsometric Formula for altitude conversion
// Altitude z = (( (P0 / P) ^ (1/5.257) - 1 ) * (T + 273.15)) / 0.0065
float temp_k = temp_c + 273.15f;
float altitude = (powf((P0 / pressure_pa), 0.190263f) - 1.0f) * temp_k / 0.0065f;

return altitude;


}

void kalman_update(kalman_alt_t *kf, float raw_alt, float dt) {
// ==========================================
// STEP 1: PREDICT STEP (Using constant velocity model)
// ==========================================
// State extrapolation: x = F * x
float pred_z = kf->z + kf->v * dt;
float pred_v = kf->v; // Constant velocity prediction

// Covariance extrapolation: P = F * P * F' + Q
// F = [1, dt]
//     [0,  1]
float P00 = kf->P[0][0] + dt * (kf->P[1][0] + kf->P[0][1]) + dt * dt * kf->P[1][1] + kf->Q_pos;
float P01 = kf->P[0][1] + dt * kf->P[1][1];
float P10 = kf->P[1][0] + dt * kf->P[1][1];
float P11 = kf->P[1][1] + kf->Q_vel;

// ==========================================
// STEP 2: UPDATE STEP (Using Baro observation)
// ==========================================
// Measurement matrix H = [1, 0] (We only measure altitude directly)
// Innovation (Residual): y = z_raw - H * x_pred
float y = raw_alt - pred_z;

// Innovation Covariance: S = H * P * H' + R
float S = P00 + kf->R_baro;

// Kalman Gain: K = P * H' * inv(S)
float K0 = P00 / S;
float K1 = P10 / S;

// Update State Estimate: x = x_pred + K * y
kf->z = pred_z + K0 * y;
kf->v = pred_v + K1 * y;

// Update Error Covariance: P = (I - K * H) * P
kf->P[0][0] = (1.0f - K0) * P00;
kf->P[0][1] = (1.0f - K0) * P01;
kf->P[1][0] = P10 - K1 * P00;
kf->P[1][1] = P11 - K1 * P01;


}