// Standard Library
#include <math.h>
#include <stdlib.h>

// Noise standard deviations
#define SIGMA_A 0.003  // Accelerometer bias noise stddev (m/s^2)
#define SIGMA_G 0.05   // Gyroscope bias noise stddev (rad/s)


// Constant
#define PI		3.141592 // Pi

double sample_noise(double mean, double stddev); // Sample Gaussian noise
void update_biasNoise();