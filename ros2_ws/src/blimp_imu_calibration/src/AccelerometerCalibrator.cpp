#include "AccelerometerCalibrator.hpp"
#include <math.h>
#include <eigen3/Eigen/Dense>


// Constructor to initialize variables
AccelerometerCalibrator::AccelerometerCalibrator(){
    initialized = false;
    sampling_rate = 50.0; // Hz
    num_samples_to_average = 32;
}


// Function to store external function calls
void AccelerometerCalibrator::init(std::function<void(float*)> func_sample_accelerometer_hardware, std::function<void(std::string)> func_print, std::function<void(int)> func_delay_ms){
    _func_sample_accelerometer_hardware = func_sample_accelerometer_hardware;
    _func_print = func_print;
    _func_delay_ms = func_delay_ms;

    if(_func_sample_accelerometer_hardware && _func_delay_ms){
        _print("Successfully initialized accelerometer calibrator.\n");
        initialized = true;
    }
}


// Function to "take a sample".
// Takes samples to estimate mean and variance, then takes many more samples and filters outliers using variance
void AccelerometerCalibrator::take_sample(){
    // Check for initialization
    if(!initialized){
        // Not initialized, cannot sample
        _print("AccelerometerCalibrator not initialized properly.\n");
        return;
    }

    // Create variable to store sample average
    Eigen::VectorXf sample_avg = Eigen::VectorXf::Zero(3);

    // Attempt to take a proper sample until success
    bool success = false;
    while(!success){
        // Reset sample
        sample_avg.setZero();

        Eigen::MatrixXf initial_samples(3, num_samples_to_average);

        // First, take (num_samples_to_average) samples to estimate the variance
        for(int sample_index=0; sample_index<num_samples_to_average; sample_index++){

            // Take a reading
            float accelerometer_reading_floats[] = {0, 0, 0};
            _func_sample_accelerometer_hardware(accelerometer_reading_floats);
            Eigen::VectorXf accelerometer_reading = Eigen::Map<Eigen::VectorXf>(accelerometer_reading_floats, 3);

            // Store reading
            initial_samples.col(sample_index) = accelerometer_reading;

            // Delay to sample at (sampling_rate) Hz
            int delay_duration_ms = 1000 / sampling_rate;
            _func_delay_ms(delay_duration_ms);
        }

        // Calculate mean
        Eigen::VectorXf mean = initial_samples.rowwise().mean();

        // Center data
        Eigen::MatrixXf initial_samples_centered = initial_samples.colwise() - mean;

        // Calculate variance
        Eigen::VectorXf variance = initial_samples_centered.array().square().rowwise().sum() / (num_samples_to_average - 1);

        // _print("mean: [");
        // for(int i=0; i<3; i++) _print(std::to_string(mean(i)) + ", ");
        // _print("],  variance: [");
        // for(int i=0; i<3; i++) _print(std::to_string(variance(i)) + ", ");
        // _print("],  std_dev: [");
        // for(int i=0; i<3; i++) _print(std::to_string(sqrt(variance(i))) + ", ");
        // _print("]\n\n");

        // Starting collecting real samples, but filter out outliers using the calculated variance
        // Track success rate and start over if we get too many fails
        int success_count = 0;
        int fail_count = 0;

        while(success_count < num_samples_to_average){

            // Take a reading
            float accelerometer_reading_floats[] = {0, 0, 0};
            _func_sample_accelerometer_hardware(accelerometer_reading_floats);
            Eigen::VectorXf accelerometer_reading = Eigen::Map<Eigen::VectorXf>(accelerometer_reading_floats, 3);

            // Calculate error
            Eigen::VectorXf error = accelerometer_reading - mean;

            // Define number of standard deviations samples must lie in
            float good_num_std = 3;

            // Check to see if sample is good
            bool good_sample = abs(error(0)) <= good_num_std * sqrt(variance(0))
                            && abs(error(1)) <= good_num_std * sqrt(variance(1))
                            && abs(error(2)) <= good_num_std * sqrt(variance(2));

            if(good_sample){
                // Good sample
                success_count++;
                sample_avg += accelerometer_reading;
            }else{
                // Bad sample
                fail_count++;
            }

            if ((fail_count > success_count && success_count > 10) || fail_count > num_samples_to_average) {
                // Failing too much, start over!
                _print("Sample fail.\n");
                break;
            }

            // Delay to sample at (sampling_rate) Hz
            int delay_duration_ms = 1000 / sampling_rate;
            _func_delay_ms(delay_duration_ms);
        }

        //if we got our samples, mark the success.  Otherwise we'll start over.
        if (success_count == num_samples_to_average) {
            success = true;

            sample_avg /= num_samples_to_average;

            _print("\tSample:    ");
            for(int i=0; i<3; i++){
                _print(std::to_string(sample_avg(i)));
                if(i < 2) _print(", ");
            }
            _print("\n");
            _print("\tStd. Dev.: " );
            for(int i=0; i<3; i++){
                _print(std::to_string(sqrt(variance(i))));
                if(i < 2) _print(", ");
            }
            _print("\n");
        }
    }

    // Store samples
    for(int i=0; i<3; i++) _samples.push_back(sample_avg(i));
}


// Function to clear internal std::vector of samples
void AccelerometerCalibrator::clear_samples(){
    _samples.clear();
}


// Function to push sample into internal samples
// Input:
//   sample: Eigen::VectorXf, length of 3
void AccelerometerCalibrator::push_sample(Eigen::VectorXf sample){
    _print("Pushing sample: [" + std::to_string(sample(0)) + ", " + std::to_string(sample(1)) + ", " + std::to_string(sample(2)) + "]\n");
    for(int i=0; i<3; i++) _samples.push_back(sample(i));
}


// Function to compute 6-state accelerometer calibration
// Model: (x-beta0)^2*beta3^2 + (y-beta1)^2*beta4^2 + (z-beta2)^2*beta5^2 = 1
// Returns:
//   Eigen::VectorXf: 6 betas
Eigen::VectorXf AccelerometerCalibrator::compute_calibration_6(){
    int num_samples = _samples.size()/3;

    // Init beta
    Eigen::VectorXf beta(6);
    beta << 0, 0, 0, 1, 1, 1;

    // Begin iterating
    int max_num_iterations = 40;
    float cost_previous = 1E8;
    float eps = 1E-5;
    for(int iteration = 0; iteration<max_num_iterations; iteration++){

        // Calculate Jacobian and residual
        Eigen::MatrixXf Jr(num_samples, 6);
        Eigen::VectorXf r(num_samples);

        for(int i=0; i<num_samples; i++){
            float x = _samples[3*i+0];
            float y = _samples[3*i+1];
            float z = _samples[3*i+2];

            Jr(i,0) = 2*(x - beta(0))*pow(beta(3),2);
            Jr(i,1) = 2*(y - beta(1))*pow(beta(4),2);
            Jr(i,2) = 2*(z - beta(2))*pow(beta(5),2);
            Jr(i,3) = -2*pow(x - beta(0),2)*beta(3);
            Jr(i,4) = -2*pow(y - beta(1),2)*beta(4);
            Jr(i,5) = -2*pow(z - beta(2),2)*beta(5);
            r(i) = 1 - pow(x - beta(0),2)*pow(beta(3),2) - pow(y - beta(1),2)*pow(beta(4),2) - pow(z - beta(2),2)*pow(beta(5),2);
        }

        // Calculate delta (Gauss-Newton Method)
        Eigen::VectorXf delta = -(Jr.transpose() * Jr).inverse() * Jr.transpose() * r;

        // Update beta
        beta += delta;

        // Check stopping criteria
        float cost = r.norm();
        float cost_delta = cost-cost_previous;
        cost_previous = cost;
        _print("Iteration " + std::to_string(iteration) + ", cost=" + std::to_string(cost) + ", cost_delta=" + std::to_string(cost_delta) + "\n");
        _print("\tGradient Norm=" + std::to_string((Jr.transpose() * r).norm()) + "\n");
        if(abs(cost_delta) <= eps){
            _print("Stopping.\n");
            break;
        }
    }

    _print("\n");
    for (int i = 0; i < 6; ++i) {
        _print(std::to_string(beta(i)));
        _print(" ");
    }
    _print("\n");

    return beta;
}


// Function to compute 9-state accelerometer calibration
// Model: A = [beta0, beta1, beta2;    b = [beta6;     norm(A*x - b) = 1
//             beta1, beta3, beta4;         beta7;
//             beta2, beta4, beta5];        beta8];
// Returns:
//   Eigen::VectorXf: 9 betas
Eigen::VectorXf AccelerometerCalibrator::compute_calibration_9(){
    int num_samples = _samples.size()/3;

    // Init beta
    Eigen::VectorXf beta(9);
    beta << 1, 0, 0, 1, 0, 1, 0, 0, 0;

    // Begin iterating
    int max_num_iterations = 100;
    float cost_previous = 1E8;
    float eps = 1E-5;
    for(int iteration = 0; iteration<max_num_iterations; iteration++){

        // Calculate residual, Jacobian, and Hessian
        Eigen::VectorXf r(num_samples);
        Eigen::MatrixXf Jr(num_samples, 9);
        Eigen::MatrixXf H = Eigen::MatrixXf::Zero(9,9);

        float a11 = beta(0);
        float a12 = beta(1);
        float a13 = beta(2);
        float a22 = beta(3);
        float a23 = beta(4);
        float a33 = beta(5);
        float b1 = beta(6);
        float b2 = beta(7);
        float b3 = beta(8);

        for(int i=0; i<num_samples; i++){
            float xri = _samples[3*i+0];
            float yri = _samples[3*i+1];
            float zri = _samples[3*i+2];

            float xci = a11*xri + a12*yri + a13*zri - b1;
            float yci = a12*xri + a22*yri + a23*zri - b2;
            float zci = a13*xri + a23*yri + a33*zri - b3;

            r(i) = 1 - pow(xci,2) - pow(yci,2) - pow(zci,2);

            Eigen::RowVectorXf Jri(9);
            Jri(0) = -2*xci*xri;
            Jri(1) = -2*xci*yri - 2*yci*xri;
            Jri(2) = -2*xci*zri - 2*zci*xri;
            Jri(3) = -2*yci*yri;
            Jri(4) = -2*yci*zri - 2*zci*yri;
            Jri(5) = -2*zci*zri;
            Jri(6) = 2*xci;
            Jri(7) = 2*yci;
            Jri(8) = 2*zci;
            Jr.block<1,9>(i,0) = Jri;

            Eigen::MatrixXf partial_squared(9, 9);
            partial_squared << -2*xri*xri,           -2*xri*yri,           -2*xri*zri,          0,                    0,          0, 2*xri,     0,     0,
                               -2*xri*yri, -2*yri*yri-2*xri*xri,           -2*yri*zri, -2*xri*yri,           -2*xri*zri,          0, 2*yri, 2*xri,     0,
                               -2*xri*zri,           -2*yri*zri, -2*zri*zri-2*xri*xri,          0,           -2*xri*yri, -2*xri*zri, 2*zri,     0, 2*xri,
                                        0,           -2*xri*yri,                    0, -2*yri*yri,           -2*yri*zri,          0,     0, 2*yri,     0,
                                        0,           -2*xri*zri,           -2*xri*yri, -2*yri*zri, -2*zri*zri-2*yri*yri, -2*yri*zri,     0, 2*zri, 2*yri,
                                        0,                    0,           -2*xri*zri,          0,           -2*yri*zri, -2*zri*zri,     0,     0, 2*zri,
                                    2*xri,                2*yri,                2*zri,          0,                    0,          0,    -2,     0,     0,
                                        0,                2*xri,                    0,      2*yri,                2*zri,          0,     0,    -2,     0,
                                        0,                    0,                2*xri,          0,                2*yri,      2*zri,     0,     0,    -2;

            H += 2*Jri.transpose()*Jri + 2*r(i)*partial_squared;

            if((H - H.transpose()).sum() != 0){
                _print("Non-symmetric H :(\n");
            }
        }

        // Calculate delta (Newton Method)
        Eigen::VectorXf delta = -H.inverse() * Jr.transpose() * r;

        // Update beta
        beta += delta;

        // Check stopping criteria
        float cost = r.norm();
        float cost_delta = cost-cost_previous;
        cost_previous = cost;
        _print("Iteration " + std::to_string(iteration) + ", cost=" + std::to_string(cost) + ", cost_delta=" + std::to_string(cost_delta) + "\n");
        _print("\tGradient Norm=" + std::to_string((Jr.transpose() * r).norm()) + "\n");
        if(abs(cost_delta) <= eps){
            _print("Stopping.\n");
            break;
        }
    }

    _print("\n");
    for (int i = 0; i < 9; ++i) {
        _print(std::to_string(beta(i)));
        _print(" ");
    }
    _print("\n");

    return beta;
}


// Function to print message if _func_print exists
// Input:
//   message: std::string to print
void AccelerometerCalibrator::_print(std::string message){
    if(_func_print) _func_print(message);
}