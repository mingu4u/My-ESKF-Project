#include <math.h>
#include <stdio.h>
#include <stdbool.h>

// Constants
#define DIM_STATE 15  // State vector dimension (p, v, q, b_a, b_g, b_pitot)
#define DIM_MEAS 13   // Measurement vector dimension (acc, mag, pitot, gps_pos, gps_vel)
#define GRAVITY 9.81  // Gravitational constant


// Helper macros for indexing
#define IDX_POS 0      // Position in state
#define IDX_VEL 3      // Velocity in state
#define IDX_Q 6        // Quaternion in state
#define IDX_BA 10      // Accelerometer bias in state
#define IDX_BG 13      // Gyroscope bias in state
#define IDX_BPITOT 14  // Pitot bias in state

// Structure Definitions
typedef struct {
    double dt; // Time step
    double q[4]; // Orientation quaternion [w, x, y, z]
    double p[3]; // Position
    double v[3]; // Velocity
    double b_a[3]; // Accelerometer bias
    double b_g[3]; // Gyroscope bias
} State;

typedef struct {
    double acc[3];
    double gyro[3];
    double mag[3];
    double gps_pos[3];
    double gps_vel[3];
    double pitot;
} Measurement;

// Kalman filter matrices
typedef struct {
    double F[DIM_STATE][DIM_STATE]; // State transition matrix
    double P[DIM_STATE][DIM_STATE]; // State covariance
    double Q[DIM_STATE][DIM_STATE]; // Process noise covariance
    double H[DIM_MEAS_TOTAL][DIM_STATE]; // Measurement matrix
    double R[DIM_MEAS_TOTAL][DIM_MEAS_TOTAL]; // Measurement noise covariance
    double K[DIM_STATE][DIM_MEAS_TOTAL]; // Kalman gain
} KalmanFilter;

// // State variables
// typedef struct {
//     double state[DIM_STATE];   // X vector : [p, v, q, b_a, b_g, b_pitot] or [position, velocity, quaternion, acc_bias, gyro_bias, pitot_bias]
//     double meas[DIM_MEAS];     // Z vector : [acc, mag, pitot, gps_pos, gps_vel]
//     double P[DIM_STATE][DIM_STATE];  // Covariance matrix
//     double Q[DIM_STATE][DIM_STATE];  // Process noise covariance
// } ESKF;

// Measurement noise covariance
/** 
 * @date 2024-11-23
 * @author mingu Kang
 * //TODO : 센서 데이터 시트 참고해 노이즈 공분산 데이터 찾아보기
*/
double R[DIM_MEAS][DIM_MEAS] = {
    {0.01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // Accelerometer noise
    {0, 0.01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0, 0, 0},  // Magnetometer noise
    {0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0},  // Pitot noise
    {0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0},  // GPS position noise
    {0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0},  // GPS velocity noise
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1},
};

// Function prototypes
void prediction_step(ESKF* eskf, double dt, const double acc[], const double gyro[]);
void predict_covariance(ESKF* eskf, double dt);
void quaternion_to_rotation(const double q[4], double R[3][3]);
void normalize_quaternion(double q[4]);
void cross_product_matrix(const double v[3], double skew[3][3]);

void update_step(ESKF* eskf, const double z[], const bool gps_available);
void error_reset(ESKF* eskf);

int main() {
    // Initialize ESKF
    ESKF eskf = {0};          // Initialize eskf state
    eskf.state[IDX_Q] = 1.0;  // Initialize quaternion as [1, 0, 0, 0] (no rotation)
    // bool gps_available = true; // This flag's value will be up to real GPS Sensor updating state.
    
    // Simulate some inputs
    /** 
 * @date 2024-11-23
 * @author mingu Kang
 * //TODO : 데이터셋 활용해 ESKF 사전 검증해보면 좋을듯
*/

    // IMU data (acceleration and angular velocity)
    // double acc[3] = {0.0, 0.0, -GRAVITY};  // Accelerometer data
    // double gyro[3] = {0.01, 0.02, 0.03};  // Gyroscope data

    // Time step
    // double dt = 0.01;

    // Time step
    double dt = 0.01;

    // Main loop
    for (int t = 0; t < 1000; t++) {
        // Prediction step
        prediction_step(&eskf, dt, acc, gyro);

        // Update step based on GPS availability
        update_step(&eskf, z, gps_available);

        // Error reset step
        error_reset(&eskf);

        // Simulate GPS unavailability
        gps_available = (t % 50 != 0);
    }

    return 0;
}

void prediction_step(ESKF* eskf, double dt, const double acc[], const double gyro[]) {
    // Predict state using IMU data



    // (Placeholder for integration of accelerometer and gyroscope data)
    // Update covariance using process noise (not shown for simplicity)
}

void update_step(ESKF* eskf, const double z[], const bool gps_available) {
    // Use all sensors if GPS is available
    if (gps_available) {
        // Include GPS position and velocity in update step
        // Measurement model and Kalman gain computation (not shown for simplicity)
    } else {
        // Use only accelerometer, magnetometer, and pitot sensors
        // Measurement model and Kalman gain computation (not shown for simplicity)
    }

    // Update state and covariance (details omitted for brevity)
}

void error_reset(ESKF* eskf) {
    // Reset state errors (normalize quaternion and zero out error terms)
    // Adjust covariance matrix using G matrix
    // Placeholder implementation
}
