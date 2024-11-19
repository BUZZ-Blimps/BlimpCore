#ifndef ACCELEROMETERCALIBRATOR
#define ACCELEROMETERCALIBRATOR

#include <functional>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>

class AccelerometerCalibrator{
    public:
        // Constructor to initialize variables
        AccelerometerCalibrator();

        // Function to store external function calls
        void init(std::function<void(float*)> func_sample_accelerometer_hardware, std::function<void(std::string)> func_print, std::function<void(int)> func_delay_ms);
        
        // Function to "take a sample".
        // Takes samples to estimate mean and variance, then takes many more samples and filters outliers using variance
        void take_sample();

        // Function to clear internal std::vector of samples
        void clear_samples();

        // Function to clear internal std::vector of samples
        void push_sample(Eigen::VectorXf sample);

        // Function to compute 6-state accelerometer calibration
        // Model: (x-beta0)^2*beta3^2 + (y-beta1)^2*beta4^2 + (z-beta2)^2*beta5^2 = 1
        // Returns:
        //   Eigen::VectorXf: 6 betas
        Eigen::VectorXf compute_calibration_6();

        // Function to compute 9-state accelerometer calibration
        // Model: A = [beta0, beta1, beta2;    b = [beta6;     norm(A*x - b) = 1
        //             beta1, beta3, beta4;         beta7;
        //             beta2, beta4, beta5];        beta8];
        // Returns:
        //   Eigen::VectorXf: 9 betas
        Eigen::VectorXf compute_calibration_9();
        
    private:
        // Initialization variable, used to block taking samples
        bool initialized;

        // Rate at which the accelerometer is sampled, in Hz
        float sampling_rate;

        // Number of samples read from hardware, averaged together to form 1 sample
        int num_samples_to_average;

        // External function call to read accelerometer data
        std::function<void(float*)> _func_sample_accelerometer_hardware;

        // External function call to print debug/feedback (does not add newlines)
        std::function<void(std::string)> _func_print;

        // External function to delay
        std::function<void(int)> _func_delay_ms;

        // Running vector of samples stored as [x0, y0, z0, x1, y1, z2, ...]
        std::vector<float> _samples;

        // Function to print message if _func_print exists
        // Input:
        //   message: std::string to print
        void _print(std::string message);
};

#endif