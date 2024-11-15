#ifndef ACCELEROMETERCALIBRATOR
#define ACCELEROMETERCALIBRATOR

#include <functional>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>

class AccelerometerCalibrator{
    public:
        AccelerometerCalibrator();

        void init(std::function<void(float*)> func_sample_accelerometer_hardware, std::function<void(std::string)> func_print, std::function<void(int)> func_delay_ms);
        void take_sample();
        void clear_samples();
        void compute_calibration();

        Eigen::VectorXf beta;
        
    private:
        bool initialized;
        float sampling_rate;
        // Number of samples read from hardware to estimate as one proper reading
        int num_samples_to_average;

        void _print(std::string message);

        std::function<void(float*)> _func_sample_accelerometer_hardware;
        std::function<void(std::string)> _func_print;
        std::function<void(int)> _func_delay_ms;

        std::vector<float> _samples;

};

#endif