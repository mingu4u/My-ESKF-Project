/**
 * @file eskf_main.c
 * @author mingu Kang (mingu4u@naver.com)
 * @brief Error State Kalman Filter (GPS+IMU+PITOT Fusion for more accurate Navigation)
 * @version 0.1
 * @date 2024-11-23
 * 
 * @copyright Copyright (c) 2024
 * 
 * 
 * @ Coordinate : NED (North-East-Down)
 */



// Standard Library
#include <string.h>

// Custom Library
#include "./include/imu_bias.h"
#include "./include/eskf_main.h"


// Create Instances
    State state;
    Measurement meas;
    KalmanFilter kalmanfilter;


int main() {

    initial_parameter();
     
    bool gps_available = true;  //HACK : This flag's value will be up to real GPS Sensor updating state.
    update_biasNoise();

    // for (int i = 0; i<3; i++) {
    //     printf("%f ", state.b_a[i]);
    // }

    /**
     * @date 2024-11-23
     * @author mingu Kang
     * // HACK : initial_parameter() 함수 내에서 지역변수 구조체가 전역 변수인 kalmanfilter로 잘 memcpy 됐는지 결과 확인하는 임시 디버깅 코드
     
        // for (int i = 0; i<DIM_MEAS; i++) {
        //     for (int j = 0; j<DIM_MEAS; j++) {

        //         printf("%f ", kalmanfilter.R[i][j]);
        //     }
        //     printf("\n");
        // }

        
    */










    // Simulate some inputs
    /** 
 * @date 2024-11-23
 * @author mingu Kang
 * //TODO : 데이터셋 활용해 ESKF 사전 검증해보면 좋을듯
*/

    // IMU data (acceleration and angular velocity)
    double acc[3] = {0.0, 0.0, -GRAVITY};  // Accelerometer data/
    double gyro[3] = {0.01, 0.02, 0.03};  // Gyroscope data

    // Time step
    double dt = 0.01;

    // // Main loop
    // for (int t = 0; t < 1000; t++) {
        // Prediction step
        prediction_step(&state, dt, acc, gyro);

        // Update step based on GPS availability
        update_step(&state, &meas, gps_available);

        // Error reset step
        error_reset(&state, &kalmanfilter);

        // Simulate GPS unavailability
        // gps_available = (t % 50 != 0);
    // }

    return 0;
}







void initial_parameter() {
    // 1. Initialize ESKF variables
    memset(&state, 0.0, sizeof(state));               // Initialize eskf state
    memset(&meas, 0.0, sizeof(state));                // Initialize eskf meas
    memset(&kalmanfilter, 0.0, sizeof(kalmanfilter)); // Initialize eskf Matrixs
    state.q[0] = 1.0;    // Initialize quaternion as [1, 0, 0, 0] (no rotation)

    // 2. Set Measurement noise Covariance R 
    /** 
     * @date 2024-11-23
     * @author mingu Kang
     * //TODO : 센서 데이터 시트 참고해 노이즈 공분산 데이터 찾아보기
    */

    KalmanFilter kf = {
        .R = {
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
        },

        .Q = {
            {0.001, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0.001, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0.001, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0.001, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0.001, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0.001, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.001, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.001, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.001},
         }
    };

    memcpy(&kalmanfilter, &kf, sizeof(kf));

}

void prediction_step(State* state, KalmanFilter* kalmanfilter, double dt, const double acc[], const double gyro[]) {
    // Extract state variables (k-1|k-1)
    double* position = state->p;
    double* velocity = state->v;
    double* quaternion = state->q;
    double* acc_bias = state->b_a;
    double* gyro_bias = state->b_g;

    // Step 1: Correct sensor measurements for biases
    
    double acc_corrected[3] = {acc[0] - acc_bias[0], acc[1] - acc_bias[1], acc[2] - acc_bias[2]};
    double gyro_corrected[3] = {gyro[0] - gyro_bias[0], gyro[1] - gyro_bias[1], gyro[2] - gyro_bias[2]};

    // Step 2: Update quaternion, Convert quaternion to rotation matrix
    double R_q[3][3];
    double dq[4] = {0, 0.5 * gyro_corrected[0], 0.5 * gyro_corrected[1], 0.5 * gyro_corrected[2]};
    double q_dot[4] = {0};
    
        // 2.1 Calculate q_dot 
        quaternion_product(quaternion, dq, q_dot);
        
        // 2.2 update quaternion
        for (int i = 0; i < 4; i++) {
            quaternion[i] += dq[i] * dt;
        }

        // 2.3 normalize quaternion
        normalize_quaternion(quaternion);

        // 2.4 Calculate R(q) based on Updated quaternion
        quaternion_to_rotation(quaternion, R_q);

    // Step 3: Compute acceleration in the inertial frame (R(q)*(a_acc - b_a) + g)
    double acc_inertial[3] = {
        R_q[0][0] * acc_corrected[0] + R_q[0][1] * acc_corrected[1] + R_q[0][2] * acc_corrected[2],
        R_q[1][0] * acc_corrected[0] + R_q[1][1] * acc_corrected[1] + R_q[1][2] * acc_corrected[2],
        R_q[2][0] * acc_corrected[0] + R_q[2][1] * acc_corrected[1] + R_q[2][2] * acc_corrected[2] + GRAVITY
    };

    // Step 4: Predict State (k-1|k-1 --> k|k-1)

        // 4.1 Position & Velocity
        for (int i = 0; i < 3; i++) {
            velocity[i] += acc_inertial[i] * dt;
            position[i] += velocity[i] * dt;
        }

        // 4.2 quaternion (already done at Step 2)

        // 4.3 Sensor bias (already done at main() -> update_biasNoise())


    // Step 5: Compute F, Get P, Q and Update P (k-1|k-1 -> k|k-1)            (P : Covariance Matrix)
    predict_covariance(state, kalmanfilter, R_q, acc, gyro);

}

// Function to compute quaternion product
void quaternion_product(const double q1[4], const double q2[4], double q_out[4]) {
    q_out[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
    q_out[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
    q_out[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
    q_out[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

// Function to normalize quaternion
void normalize_quaternion(double q[4]) {
    double norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (norm == 0.0) {
        // Handle invalid quaternion: reset to identity quaternion
        q[0] = 1.0;
        q[1] = 0.0;
        q[2] = 0.0;
        q[3] = 0.0;
    } 
    else {
        for (int i = 0; i < 4; i++) {
            q[i] /= norm;
        }
    }
}

// Convert quaternion to rotation matrix (R_q)
void quaternion_to_rotation(const double q[4], double R_q[3][3]) {
    double q_w = q[0], q_x = q[1], q_y = q[2], q_z = q[3];
    R_q[0][0] = 1 - 2 * (q_y * q_y + q_z * q_z);
    R_q[0][1] = 2 * (q_x * q_y - q_w * q_z);
    R_q[0][2] = 2 * (q_x * q_z + q_w * q_y);
    R_q[1][0] = 2 * (q_x * q_y + q_w * q_z);
    R_q[1][1] = 1 - 2 * (q_x * q_x + q_z * q_z);
    R_q[1][2] = 2 * (q_y * q_z - q_w * q_x);
    R_q[2][0] = 2 * (q_x * q_z - q_w * q_y);
    R_q[2][1] = 2 * (q_y * q_z + q_w * q_x);
    R_q[2][2] = 1 - 2 * (q_x * q_x + q_y * q_y);
}

// Predict the covariance matrix
void predict_covariance(State* state, KalmanFilter* kalmanfilter, double R_q[3][3], const double acc[], const double gyro[]) {
    
    // Placeholder: Compute F and Q matrices, then update P
    compute_F_matrix(state, kalmanfilter, R_q, acc, gyro);


    // Placeholder matrices for intermediate results
    double FP[DIM_STATE][DIM_STATE] = {0};
    double FPFt[DIM_STATE][DIM_STATE] = {0};
    double Ft[DIM_STATE][DIM_STATE] = {0};

    // Compute FP = F * P
    matrix_multiply(kalmanfilter->F, kalmanfilter->P, FP);

    // Compute Ft = F^T
    matrix_transpose(kalmanfilter->F, Ft);

    // Compute FPFt = FP * F^T
    matrix_multiply(FP, Ft, FPFt);

    // Update P = F * P * F^T + Q
    for (int i = 0; i < DIM_STATE; i++) {
        for (int j = 0; j < DIM_STATE; j++) {
            kalmanfilter->P[i][j] = FPFt[i][j] + kalmanfilter->Q[i][j];
        }
    }

}

void compute_F_matrix(State* state, KalmanFilter* kalmanfilter, double R_q[3][3], const double acc[], const double gyro[]) {

    double F[DIM_STATE][DIM_STATE];
    memset(F, 0.0, sizeof(double) * DIM_STATE * DIM_STATE);
    
    
    // Position derivatives (dp/dv)
    for (int i = 0; i < 3; i++) {
        F[i][i + 3] = 1.0; // dp/dv = 1
    }

    // Velocity derivatives (dv/dq and acceleration bias)
    double ax = acc[0];
    double ay = acc[1];
    double az = acc[2];

    double a_sqew[3][3] =   {
                            {0.0, -az, ay},
                            {az, 0.0, -ax},
                            {-ay, ax, 0.0}
                            };
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            // F[i + 3][j + 6] = 0.0;
            for (int k = 0; k < 3; k++) {
                F[i + 3][j + 6] += -R_q[i][k] * a_sqew[k][j];
            }            
            F[i + 3][j + 9] = -R_q[i][j];
        }
    }   

    // Quaternion derivatives 
    double wx = gyro[0];
    double wy = gyro[1];    
    double wz = gyro[2];

    double w_sqew[3][3] =   {
                            {0.0, -wz, wy},
                            {wz, 0.0, -wx},
                            {-ay, wx, 0.0}
                            };    

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            F[i + 6][j + 6] = -w_sqew[i][j]; // dq/
        }
        F[i + 6][i + 12] = -1.0; // dq/db_g
    }
    
    memcpy(&kalmanfilter->F, &F, sizeof(F));

}



// Multiply two matrices (A * B)
void matrix_multiply(double A[DIM_STATE][DIM_STATE], double B[DIM_STATE][DIM_STATE], double result[DIM_STATE][DIM_STATE]) {
    for (int i = 0; i < DIM_STATE; i++) {
        for (int j = 0; j < DIM_STATE; j++) {
            result[i][j] = 0.0;
            for (int k = 0; k < DIM_STATE; k++) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// Transpose a matrix
void matrix_transpose(double A[DIM_STATE][DIM_STATE], double result[DIM_STATE][DIM_STATE]) {
    for (int i = 0; i < DIM_STATE; i++) {
        for (int j = 0; j < DIM_STATE; j++) {
            result[i][j] = A[j][i];
        }
    }
}










void update_step(State* state, const Measurement* meas, const bool gps_available) {
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

void error_reset(State* state, KalmanFilter *kalmanfilter) {
    // Reset state errors (normalize quaternion and zero out error terms)
    // Adjust covariance matrix using G matrix
    // Placeholder implementation
}
