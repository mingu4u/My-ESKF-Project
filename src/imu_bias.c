
// Custom Library
#include "./include/imu_bias.h"
#include "./include/eskf_main.h"
#include <stdint.h>

// Create Instances
State state;

// Sample Gaussian noise
double sample_noise(double mean, double stddev) {
    double u1 = (double)rand() / RAND_MAX;
    double u2 = (double)rand() / RAND_MAX;
    double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
    return z0 * stddev + mean;
}

void update_biasNoise() {
    // Simulate noise for each axis
        // Accelometer Bias Noise [m/s^2\n]
        uint8_t DIM_BIAS = 3;
        double w_a[3];
        // double w_a_x = sample_noise(0.0, SIGMA_A); 
        // double w_a_y = sample_noise(0.0, SIGMA_A);
        // double w_a_z = sample_noise(0.0, SIGMA_A);
        for (int i = 0; i < DIM_BIAS; i++) {
                w_a[i] = sample_noise(0.0, SIGMA_A);  // Sample from N(0, SIGMA_A^2)
                state.b_a[i] += w_a[i];
        }
        // Gyro Bias Noise [rad/s]
        double w_g[3];
        // double w_g_x = sample_noise(0.0, SIGMA_G);
        // double w_g_y = sample_noise(0.0, SIGMA_G);
        // double w_g_z = sample_noise(0.0, SIGMA_G);
        for (int i = 0; i < DIM_BIAS; i++) {
            w_g[i] = sample_noise(0.0, SIGMA_G);  // Sample from N(0, SIGMA_G^2)
            state.b_g[i] += w_g[i];
        }
}

// int main() {
//     // Simulate noise for each axis
//     double w_a_x = sample_noise(0.0, SIGMA_A);
//     double w_a_y = sample_noise(0.0, SIGMA_A);
//     double w_a_z = sample_noise(0.0, SIGMA_A);

//     double w_g_x = sample_noise(0.0, SIGMA_G);
//     double w_g_y = sample_noise(0.0, SIGMA_G);
//     double w_g_z = sample_noise(0.0, SIGMA_G);

//     printf("Accelerometer bias noise: [%f, %f, %f] m/s^2\n", w_a_x, w_a_y, w_a_z);
//     printf("Gyroscope bias noise: [%f, %f, %f] rad/s\n", w_g_x, w_g_y, w_g_z);

//     return 0;
// }
