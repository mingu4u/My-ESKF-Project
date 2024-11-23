// Standard Library
#include <math.h>
#include <stdio.h>
#include <stdbool.h>

// Constants
#define DIM_STATE 15  // State vector dimension (p, v, q, b_a, b_g, b_pitot)
#define DIM_MEAS 13   // Measurement vector dimension (acc, mag, pitot, gps_pos, gps_vel)
#define GRAVITY 9.81  // Gravitational constant


// // Helper macros for indexing
// XXX : 삭제 예정
// #define IDX_POS 0      // Position in state
// #define IDX_VEL 3      // Velocity in state
// #define IDX_Q 6        // Quaternion in state
// #define IDX_BA 10      // Accelerometer bias in state
// #define IDX_BG 13      // Gyroscope bias in state
// #define IDX_BPITOT 14  // Pitot bias in state

#pragma pack(push, 1)
// Structure Definitions
double dt; //Time step
typedef struct {

    double p[3]; //Position
    double v[3]; //Velocity
    double q[4]; //Orientation quaternion [w, x, y, z]
    double b_a[3]; //Accelerometer bias
    double b_g[3]; //Gyroscope bias
    double b_p; // Pitot bias
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
    double H[DIM_MEAS][DIM_STATE]; // Measurement matrix
    double R[DIM_MEAS][DIM_MEAS]; // Measurement noise covariance
    double K[DIM_STATE][DIM_MEAS]; // Kalman gain
} KalmanFilter;

#pragma pack(pop)

// Create Instances
    extern State state;
    extern Measurement meas;
    extern KalmanFilter kalmanfilter;

// Function prototypes
void initial_parameter();
void prediction_step(State* state, double dt, const double acc[], const double gyro[]);
void predict_covariance(State* state, KalmanFilter kalmanfilter, double dt);
void quaternion_to_rotation(const double q[4], double R_q[3][3]);
void normalize_quaternion(double q[4]);

void cross_product_matrix(const double v[3], double skew[3][3]);
void quaternion_product(const double q1[4], const double q2[4], double q_out[4]);
void update_step(State* state, const Measurement* meas, const bool gps_available);
void error_reset(State* state, KalmanFilter* kalmanfilter);
