#ifndef ACCELEROMETERCALIBRATOR
#define ACCELEROMETERCALIBRATOR

#include <functional>
#include <string>
#include <vector>

class AccelerometerCalibrator{
    public:
        AccelerometerCalibrator();

        void init(std::function<void(long*)> func_sample_accelerometer_hardware, std::function<void(std::string)> func_print, std::function<void(int)> func_delay_ms);
        void take_sample();
        void clear_samples();
        void compute_calibration();
        
        //parameters for model.  beta[0], beta[1], and beta[2] are the 0-G marks (about 512)
        double beta[6] = {0,0,0,1,1,1};                        
        // while beta[3], beta[4], and beta[5] are the scaling factors.  So, e.g., if xpin reads
        // value x, number of G's in the x direction in beta[3]*(x - beta[0]).

    private:
        bool initialized;
        int sampling_rate;

        void _print(std::string message);

        std::function<void(long*)> _func_sample_accelerometer_hardware;
        std::function<void(std::string)> _func_print;
        std::function<void(int)> _func_delay_ms;

        std::vector<float> _samples;

        //matrices for Gauss-Newton computations
        double JS[6][6];
        double dS[6];
        double delta[6];

        void reset_calibration_matrices();
        void update_calibration_matrices(const float *data);
        void compute_calibration_matrices();
        void find_delta();
        void calibrate_model();

        // Number of samples read from hardware to estimate as one proper reading
        int num_samples_to_average;

};

#endif