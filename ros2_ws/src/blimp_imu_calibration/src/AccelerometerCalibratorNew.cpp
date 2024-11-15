#include "AccelerometerCalibratorNew.hpp"
#include <math.h>
#include <eigen3/Eigen/Dense>

AccelerometerCalibrator::AccelerometerCalibrator(){
    initialized = false;
    sampling_rate = 50; // Hz
    num_samples_to_average = 32;
}

void AccelerometerCalibrator::init(std::function<void(long*)> func_sample_accelerometer_hardware, std::function<void(std::string)> func_print, std::function<void(int)> func_delay_ms){
    _func_sample_accelerometer_hardware = func_sample_accelerometer_hardware;
    _func_print = func_print;
    _func_delay_ms = func_delay_ms;

    if(_func_sample_accelerometer_hardware && _func_delay_ms){
        _print("Successfully initialized accelerometer calibrator.\n");
        initialized = true;
    }
}

void AccelerometerCalibrator::take_sample(){
    // Check for initialization
    if(!initialized){
        // Not initialized, cannot sample
        _print("AccelerometerCalibrator not initialized properly.\n");
        return;
    }

    // Create variable to store sample
    float sample_out[] = {0, 0, 0};

    // Attempt to take a proper sample until success
    bool success = false;
    while(!success){

        float mean[] = {0, 0, 0};
        float sum_squared_deviations[] = {0, 0, 0};
        float variance[] = {0, 0, 0};

        // First, take (num_samples_to_average) samples to estimate the variance
        // Make all variables longs because ints will overflow
        for(int i=0; i<num_samples_to_average; i++){

            // Take a reading
            long accelerometer_reading[] = {0, 0, 0};
            _func_sample_accelerometer_hardware(accelerometer_reading);
            float accelerometer_reading_float[] = {0,0,0};
            for(int j=0; j<3; j++) accelerometer_reading_float[j] = *((float*)(&(accelerometer_reading[j])));
            // _func_print("sample: " + std::to_string(accelerometer_reading[0]));

            // Store reading in terms
            float x = accelerometer_reading_float[0];
            float y = accelerometer_reading_float[1];
            float z = accelerometer_reading_float[2];
            
            // Welford's algorithm
            for(int j=0; j<3; j++){
                float a = accelerometer_reading_float[j];
                mean[j] += (a-mean[j])/(i+1);
                sum_squared_deviations[j] += (a-mean[j])*(a-mean[j]);
            }

            // // Increment sums by terms
            // sum[0] += x;
            // sum[1] += y;
            // sum[2] += z;

            // // Increment sums of squares by terms squared 
            // sum_squares[0] += pow(x,2);
            // sum_squares[1] += pow(y,2);
            // sum_squares[2] += pow(z,2);

            // Delay to sample at (sampling_rate) Hz
            int delay_duration_ms = 1000 / sampling_rate;
            _func_delay_ms(delay_duration_ms);

            // _print("x=" + std::to_string(x) + ",  sum[0]=" + std::to_string(sum[0]) + ",  sum_squares[0]=" + std::to_string(sum_squares[0]) + ",  sum_squared[0]=" + std::to_string(pow(sum[0],2)) + "\n");
            // _print("x=" + std::to_string(x) + ",  mean[0]=" + std::to_string(mean[0]) + ",  sum_squared_deviations[0]=" + std::to_string(sum_squared_deviations[0]) + "\n");
            // for(int j=0; j<3; j++) _print(std::to_string(j)+"="+std::to_string(accelerometer_reading_float[j])+",  mean="+std::to_string(mean[j])+"\n");
        }
        // for(int i=0; i<3; i++) variance[i] = sum_squared_deviations[i]/(num_samples_to_average - 1);

        // Calculate variances
        for (int i = 0; i < 3; i++) {

            // float sum_squared = pow(sum[i],2);
            // variance[i] = sum_squares[i] / (num_samples_to_average - 1) - sum_squared / (num_samples_to_average - 1);
            variance[i] = sum_squared_deviations[i]/(num_samples_to_average - 1);

            // _print("Sum squares: ");
            // _print(std::to_string(sum_squares[i]));
            // _print(" Sum, squared: ");
            // _print(std::to_string(sum_squared));
            // _print(" Variance: ");
            // _print(std::to_string(variance[i]));
            // _print("\n");
        }

        // Starting collecting real samples, but filter out outliers using the calculated variance
        // Track success rate and start over if we get too many fails
        int success_count = 0;
        int fail_count = 0;

        while(success_count < num_samples_to_average){

            // Take a reading
            long accelerometer_reading[] = {0, 0, 0};
            _func_sample_accelerometer_hardware(accelerometer_reading);
            float accelerometer_reading_float[] = {0,0,0};
            for(int i=0; i<3; i++) accelerometer_reading_float[i] = *((float*)(&(accelerometer_reading[i])));

            // Store reading in terms
            float x = accelerometer_reading_float[0];
            float y = accelerometer_reading_float[1];
            float z = accelerometer_reading_float[2];

            // Filter out outliers using calculated variance
            // float dx = x - sum[0]/num_samples_to_average;
            // float dy = y * sum[1]/num_samples_to_average;
            // float dz = z * sum[2]/num_samples_to_average;
            float dx = x - mean[0];
            float dy = y - mean[1];
            float dz = z - mean[2];

            // Number of standard deviations samples must lie within
            long good_num_std = 3;

            // _print("dx(" + std::to_string(abs(dx)) + ")<3sig(" + std::to_string(good_num_std*sqrt(variance[0])) + ") = "+std::to_string(abs(dx) < good_num_std*sqrt(variance[0]))+"\n");
            // _print("dy(" + std::to_string(abs(dy)) + ")<3sig(" + std::to_string(good_num_std*sqrt(variance[1])) + ") = "+std::to_string(abs(dy) < good_num_std*sqrt(variance[1]))+"\n");
            // _print("dz(" + std::to_string(abs(dz)) + ")<3sig(" + std::to_string(good_num_std*sqrt(variance[2])) + ") = "+std::to_string(abs(dz) < good_num_std*sqrt(variance[2]))+"\n");

            // Check to see if sample is good
            if(abs(dx) < good_num_std*sqrt(variance[0])
            && abs(dy) < good_num_std*sqrt(variance[1])
            && abs(dz) < good_num_std*sqrt(variance[2])){
                // Good sample
                success_count++;
                sample_out[0] += x;
                sample_out[1] += y;
                sample_out[2] += z;
                // _print("good, ");

            }else{
                // Bad sample
                fail_count++;
                // _print("bad, ");
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

            sample_out[0] /= num_samples_to_average;
            sample_out[1] /= num_samples_to_average;
            sample_out[2] /= num_samples_to_average;

            _print("# ");
            _print(std::to_string(sample_out[0]));
            _print(" ");
            _print(std::to_string(sample_out[1]));
            _print(" ");
            _print(std::to_string(sample_out[2]));
            _print(";\n");
        }
    }

    // Store samples
    _samples.push_back(sample_out[0]);
    _samples.push_back(sample_out[1]);
    _samples.push_back(sample_out[2]);
}

void AccelerometerCalibrator::clear_samples(){
    _samples.clear();
}

void AccelerometerCalibrator::compute_calibration(){
    // calibrate_model();

    int num_samples = _samples.size()/3;

    // Init beta
    for(int i=0; i<3; i++) beta[i] = 0;
    for(int i=3; i<6; i++) beta[i] = 1;

    // Begin iterating
    int max_num_iterations = 40;
    float cost_previous = 1E8;
    float eps = 1E-5;
    for(int iteration = 0; iteration<max_num_iterations; iteration++){

        // Calculate Jacobian and residual
        Eigen::MatrixXd Jr(num_samples, 6);
        Eigen::VectorXd r(num_samples);

        for(int i=0; i<num_samples; i++){
            float x = _samples[3*i+0];
            float y = _samples[3*i+1];
            float z = _samples[3*i+2];

            Jr(i,0) = 2*(x - beta[0])*pow(beta[3],2);
            Jr(i,1) = 2*(y - beta[1])*pow(beta[4],2);
            Jr(i,2) = 2*(z - beta[2])*pow(beta[5],2);
            Jr(i,3) = -2*pow(x - beta[0],2)*beta[3];
            Jr(i,4) = -2*pow(y - beta[1],2)*beta[4];
            Jr(i,5) = -2*pow(z - beta[2],2)*beta[5];
            r(i) = 1 - pow(x - beta[0],2)*pow(beta[3],2) - pow(y - beta[1],2)*pow(beta[4],2) - pow(z - beta[2],2)*pow(beta[5],2);
        }

        // Calculate delta
        Eigen::VectorXd delta = -(Jr.transpose() * Jr).inverse() * Jr.transpose() * r;

        // Update beta
        for(int i=0; i<6; i++) beta[i] += delta(i);


        // Check stopping criteria
        float cost = r.norm();
        float cost_delta = cost-cost_previous;
        cost_previous = cost;
        _print("Iteration " + std::to_string(iteration) + ", cost=" + std::to_string(cost) + ", cost_delta=" + std::to_string(cost_delta) + "\n");
        if(abs(cost_delta) <= eps){
            _print("Stopping.\n");
            break;
        }
    }

    _print("\n");
    for (int i = 0; i < 6; ++i) {
        _print(std::to_string(beta[i]));
        _print(" ");
    }
    _print("\n");
}

void AccelerometerCalibrator::_print(std::string message){
    if(_func_print) _func_print(message);
}



//Gauss-Newton functions
void AccelerometerCalibrator::reset_calibration_matrices() {
    int j, k;
    for (j = 0; j < 6; ++j) {
        dS[j] = 0.0;
        for (k = 0; k < 6; ++k) {
            JS[j][k] = 0.0;
        }
    }
}

void AccelerometerCalibrator::update_calibration_matrices(const float *data) {
    int j, k;
    double dx, b;
    double residual = 1.0;
    double jacobian[6];

    for (j = 0; j < 3; ++j) {
        b = beta[3 + j];
        dx = ((float)data[j]) - beta[j];
        residual -= b * b * dx * dx;
        jacobian[j] = 2.0 * b * b * dx;
        jacobian[3 + j] = -2.0 * b * dx * dx;
    }

    for (j = 0; j < 6; ++j) {
        dS[j] += jacobian[j] * residual;
        for (k = 0; k < 6; ++k) {
            JS[j][k] += jacobian[j] * jacobian[k];
        }
    }
}

void AccelerometerCalibrator::compute_calibration_matrices() {
    reset_calibration_matrices();
    int num_samples = _samples.size()/3;
    for (int i = 0; i < num_samples; i++) {
        float data[] = {_samples[3*i+0], _samples[3*i+1], _samples[3*i+2]};
        update_calibration_matrices(data);
    }
}

void AccelerometerCalibrator::find_delta() {
    //Solve 6-d matrix equation JS*x = dS
    //first put in upper triangular form
    int i, j, k;
    double mu;

    //make upper triangular
    for (i = 0; i < 6; ++i) {
        //eliminate all nonzero entries below JS[i][i]
        for (j = i + 1; j < 6; ++j) {
            mu = JS[i][j] / JS[i][i];
            if (mu != 0.0) {
                dS[j] -= mu * dS[i];
                for (k = j; k < 6; ++k) {
                    JS[k][j] -= mu * JS[k][i];
                }
            }
        }
    }

    //back-substitute
    for (i = 5; i >= 0; --i) {
        dS[i] /= JS[i][i];
        JS[i][i] = 1.0;
        for (j = 0; j < i; ++j) {
            mu = JS[i][j];
            dS[j] -= mu * dS[i];
            JS[i][j] = 0.0;
        }
    }

    for (i = 0; i < 6; ++i) {
        delta[i] = dS[i];
    }
}

void AccelerometerCalibrator::calibrate_model() {
    double eps = 0.000000001;
    int num_iterations = 40;
    double change = 100.0;
    for(int iteration=0; iteration<num_iterations; iteration++){
        // Check stopping critereon
        if(change <= eps){
            break;
        }

        compute_calibration_matrices();
        find_delta();
        change = delta[0] * delta[0] + delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2] + delta[3] * delta[3] / (beta[3] * beta[3]) + delta[4] * delta[4] / (beta[4] * beta[4]) + delta[5] * delta[5] / (beta[5] * beta[5]);

        for (int i = 0; i < 6; ++i) {
            beta[i] -= delta[i];
        }

        reset_calibration_matrices();
    }

    _print("\n");
    for (int i = 0; i < 6; ++i) {
        _print(std::to_string(beta[i]));
        _print(" ");
    }
    _print("\n");
}
