#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <math.h>
#include <fstream>
#include <iomanip>
#include <random>

#include "OPI_IMU.hpp"
#include "AccelerometerCalibrator.hpp"

#define SUPPRESS_UNUSED_WARNING(parameter) (void) parameter;


OPI_IMU imu;
AccelerometerCalibrator calibrator;
std::default_random_engine rng;


// Function to read accelerometer data
// Input:
//   data: pointer to float array, first 3 values will be set to accelerometer x, y, z
void func_sample_accelerometer_hardware(float* data){
    imu.IMU_read();
    data[0] = imu.AccXraw;
    data[1] = imu.AccYraw;
    data[2] = imu.AccZraw;
}


// Function to print debug/feedback (does not add newlines)
// Input:
//   message: std::string to be printed
void func_print(std::string message){
    std::cout << message;
}

// Function to delay
// Input:
//   delay_length_ms: int with number of milliseconds to be delayed by
void func_delay_ms(int delay_length_ms){
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_length_ms));
}

// Function to save betas to yaml file
// Input:
//   file_path: std::string, absolute file path to save yaml file at
//   beta: Eigen::VectorXf containing betas to be written to file
void save_to_yaml(std::string file_path, Eigen::VectorXf beta){
    // Open YAML file
    std::ofstream yamlFile(file_path);
    if (!yamlFile.is_open()) {
        std::cerr << "Error opening file for saving." << std::endl << "File Path: " << file_path << std::endl;
        return;
    }

    // Write betas
    yamlFile << "betas: [";
    for(int i=0; i<beta.size(); i++){
        yamlFile << std::fixed << std::setprecision(16) << beta(i);
        if(i < beta.size() - 1){
            yamlFile << ", ";
        }
    }
    yamlFile << "]";

    // Close the file
    yamlFile.close();
}

// Function to generate fake accelerometer samples.
// Randomly samples sphere, contorts with A and b, adds mean-zero gaussian noise
// Inputs:
//   A_true: Eigen::MatrixXf, 3x3 symmetric matrix
//   b_true: Eigen::VectorXf, length of 3
//   variance: float, variance of gaussian noise
// Returns:
//   Eigen::VectorXf: fake accelerometer sample
Eigen::VectorXf generate_accelerometer_sample(Eigen::MatrixXf A_true, Eigen::VectorXf b_true, float variance){
    std::uniform_real_distribution<float> dist_uniform(0.0, 1.0);
    std::normal_distribution<float> dist_normal(0, variance);

    // Generate point on sphere
    Eigen::VectorXf X_spherical(3);
    float phi = 2*M_PI*dist_uniform(rng);
    float theta = 2*M_PI*dist_uniform(rng);
    X_spherical(0) = 1*sin(theta)*cos(phi);
    X_spherical(1) = 1*sin(theta)*sin(phi);
    X_spherical(2) = 1*cos(theta);

    // Contort using parameters
    Eigen::MatrixXf X_raw = A_true.inverse() * (X_spherical.colwise() + b_true);

    // Add noise
    for(int i=0; i<3; i++){
        X_raw(i) += dist_normal(rng);
    }

    return X_raw;
}

// "main" for taking real samples and calibrating an accelerometer
int main_real(int argc, char* argv[]){
    // Suppress "unused parameter" warnings
    SUPPRESS_UNUSED_WARNING(argc);

    // Get user input for blimp name
    std::cout << "What blimp is being calibrated? ";
    std::string blimp_name;
    std::cin >> blimp_name;

    imu.OPI_IMU_Setup();
    calibrator.init(func_sample_accelerometer_hardware, func_print, func_delay_ms);

    std::cout << "Beginning calibration of " << blimp_name << "." << std::endl;
    std::cout << "To take a new sample, put the IMU in a new rotation and press ENTER when it is still. DO NOT MOVE IT." << std::endl;
    std::cout << "To stop sampling, type 'hawk tuah' and press ENTER." << std::endl;
    std::cin.ignore();

    int sample_count = 0;
    bool sampling = true;
    while(sampling){
        
        sample_count++;
        std::cout << "Sample " << std::to_string(sample_count) << ": ";
        
        // Get user input
        std::string input;
        std::getline(std::cin, input);
        if(input == ""){
            // Pressed ENTER, take sample
            calibrator.take_sample();

        }else if(input == "hawk tuah"){
            // Typed something, stop sampling
            sampling = false;
        }else{
            std::cout << "Non-empty, non-'hawk tuah' input detected. Please press ENTER to take a sample or 'hawk tuah' to stop sampling." << std::endl;
        }
    }

    std::cout << "All samples taken. Beginning calibration." << std::endl;
    Eigen::VectorXf beta = calibrator.compute_calibration_9();
    std::cout << "Calibration complete." << std::endl;

    // Construct YAML file path
    std::string arg0 = argv[0];
    std::string file_path;

    std::cout << arg0 << std::endl;
    std::cout << arg0.find("/root/") << std::endl;
    std::cout << (arg0.find("/root/") == 0) << std::endl;

    if (arg0.find("/root/") == 0){
        file_path = "/root/";
    }else{
        file_path = "/home/opi/";
    }
    file_path += blimp_name + "_accel_cal.yaml";

    // Save beta to YAML
    std::cout << "Saving betas to YAML at " << file_path << std::endl;
    save_to_yaml(file_path, beta);

    // Generate A and b from beta
    Eigen::MatrixXf A(3,3);
    Eigen::VectorXf b(3);
    if(beta.size() == 9){
        A << beta(0), beta(1), beta(2),
             beta(1), beta(3), beta(4),
             beta(2), beta(4), beta(5);
        b << beta(6), beta(7), beta(8);
    }else if(beta.size() == 6){
        A << beta(3),       0,       0,
                   0, beta(4),       0,
                   0,       0, beta(5);
        b << beta(3) * beta(0), beta(4) * beta(1), beta(5) * beta(2);
    }else{
        // Error
        std::cout << "Error: Invalid number of betas (" << beta.size() << ")." << std::endl;
        return 1;
    }

    // Show live calibration results
    std::cout << "Showing live calibrated accelerometer samples." << std::endl;
    while(true){
        // Sample IMU
        float sample_floats[] = {0, 0, 0};
        func_sample_accelerometer_hardware(sample_floats);
        Eigen::VectorXf sample = Eigen::Map<Eigen::VectorXf>(sample_floats, 3);

        // Correct sample
        Eigen::VectorXf corrected_sample = A*sample - b;

        // Calculate norms
        float norm = sample.norm();
        float corrected_norm = corrected_sample.norm();

        // Print corrected samples and norms
        std::cout << "[" << std::to_string(sample(0)) << ", " << std::to_string(sample(1)) << ", " <<std::to_string(sample(2)) << "], norm^2 = " << std::to_string(norm) << ",\t\t";
        std::cout << "[" << std::to_string(corrected_sample(0)) << ", " << std::to_string(corrected_sample(1)) << ", " <<std::to_string(corrected_sample(2)) << "], norm^2 = " << std::to_string(corrected_norm) << std::endl;

        // Delay
        func_delay_ms(500);
    }

    return 0;
}

// "main" for generating fake samples from a fake accelerometer, to test 9-state calibration
int main_test(int argc, char* argv[]){
    // Suppress "unused parameter" warnings
    SUPPRESS_UNUSED_WARNING(argc);
    SUPPRESS_UNUSED_WARNING(argv);

    // Init calibrator
    calibrator.init(func_sample_accelerometer_hardware, func_print, func_delay_ms);

    // Define true A, b, beta
    Eigen::MatrixXf A_true(3,3);
    A_true <<    1,  0.2, 0.15,
               0.2, 0.87, 0.07,
              0.15, 0.07,  1.1;
    Eigen::VectorXf b_true(3);
    b_true << 0.01, 0.07, 0.02;

    // Construct true beta vector
    Eigen::VectorXf beta_true(9);
    beta_true << A_true(0,0), A_true(0,1), A_true(0,2), A_true(1,1), A_true(1,2), A_true(2,2), b_true(0), b_true(1), b_true(2);

    // Define number of samples
    int N = 100;

    // Push samples
    calibrator.clear_samples();
    for(int i=0; i<N; i++){
        calibrator.push_sample(generate_accelerometer_sample(A_true, b_true, pow(0.000001,2)));
    }

    // Run calibration
    std::cout << "All samples taken. Beginning calibration." << std::endl;
    Eigen::VectorXf beta = calibrator.compute_calibration_9();
    std::cout << "Calibration complete." << std::endl;

    // Print true vs estimated betas
    std::cout << "True beta:" << std::endl << beta_true.transpose() << std::endl << std::endl;
    std::cout << "Estimated beta:" << std::endl << beta.transpose() << std::endl << std::endl;

    return 0;
}

int main(int argc, char* argv[]){
    // Comment/uncomment the following lines to run calibration on real samples or test samples

    return main_real(argc, argv);
    // return main_test(argc, argv);
}