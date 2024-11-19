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

// OPI_IMU imu;
AccelerometerCalibrator calibrator;

void func_sample_accelerometer_hardware(float* data){
    // imu.IMU_read();
    // data[0] = imu.AccXraw;
    // data[1] = imu.AccYraw;
    // data[2] = imu.AccZraw;
}


void func_print(std::string message){
    std::cout << message;
}


void func_delay_ms(int delay_length_ms){
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_length_ms));
}


void saveToYAML(std::string blimp_name, Eigen::VectorXf betas){
    // Define file path
    std::string file_path = "/home/opi/" + blimp_name + "_accel_cal.yaml";

    // Open YAML file
    std::ofstream yamlFile(file_path);
    if (!yamlFile.is_open()) {
        std::cerr << "Error opening file for saving." << std::endl << "File Path: " << file_path << std::endl;
        return;
    }

    // Write betas
    yamlFile << "betas: [";
    for(int i=0; i<betas.size(); i++){
        yamlFile << std::fixed << std::setprecision(16) << betas(i);
        if(i < betas.size() - 1){
            yamlFile << ", ";
        }
    }
    yamlFile << "]";

    // Close the file
    yamlFile.close();
}


int main(){
    // std::cout << "What blimp is being calibrated? ";
    // std::string blimp_name;
    // std::cin >> blimp_name;

    // imu.OPI_IMU_Setup();
    calibrator.init(func_sample_accelerometer_hardware, func_print, func_delay_ms);

    // std::cout << "Beginning calibration of " << blimp_name << "." << std::endl;
    // std::cout << "To take a new sample, put the IMU in a new rotation and press ENTER when it is still. DO NOT MOVE IT." << std::endl;
    // std::cout << "To stop sampling, type 'hawk tuah' and press ENTER." << std::endl;
    // std::cin.ignore();

    // int sample_count = 0;
    // bool sampling = true;
    // while(sampling){
        
    //     sample_count++;
    //     std::cout << "Sample " << std::to_string(sample_count) << ": ";
        
    //     // Get user input
    //     std::string input;
    //     std::getline(std::cin, input);
    //     if(input == ""){
    //         // Pressed ENTER, take sample
    //         calibrator.take_sample();

    //     }else if(input == "hawk tuah"){
    //         // Typed something, stop sampling
    //         sampling = false;
    //     }else{
    //         std::cout << "Non-empty, non-'hawk tuah' input detected. Please press ENTER to take a sample or 'hawk tuah' to stop sampling." << std::endl;
    //     }
    // }

    // Define true A, b, beta
    Eigen::MatrixXf A_true(3,3);
    A_true <<    1,  0.2, 0.15,
               0.2, 0.87, 0.07,
              0.15, 0.07,  1.1;
    Eigen::VectorXf b_true(3);
    b_true << 0.01, 0.07, 0.02;
    Eigen::VectorXf beta_true(9);
    beta_true << A_true(0,0), A_true(0,1), A_true(0,2), A_true(1,1), A_true(1,2), A_true(2,2), b_true(0), b_true(1), b_true(2);

    // Define number of samples
    int N = 100;

    std::default_random_engine rng;
    std::uniform_real_distribution<float> dist_uniform(0.0, 1.0);
    std::normal_distribution<float> dist_normal(0, pow(0.000001,2));

    // Generate points on sphere
    Eigen::MatrixXf X_spherical(3, N);
    for(int i=0; i<N; i++){
        float phi = 2*M_PI*dist_uniform(rng);
        float theta = 2*M_PI*dist_uniform(rng);
        X_spherical(0,i) = 1*sin(theta)*cos(phi);
        X_spherical(1,i) = 1*sin(theta)*sin(phi);
        X_spherical(2,i) = 1*cos(theta);
    }

    // Contort using parameters
    Eigen::MatrixXf X_raw = A_true.inverse() * (X_spherical.colwise() + b_true);

    // Add noise
    for(int i=0; i<N; i++){
        for(int j=0; j<3; j++){
            X_raw(j,i) += dist_normal(rng);
        }
    }

    // Push samples
    calibrator.clear_samples();
    for(int i=0; i<N; i++){
        calibrator.push_sample(X_raw.col(i));
    }

    std::cout << "All samples taken. Beginning calibration." << std::endl;
    Eigen::VectorXf beta = calibrator.compute_calibration_9();
    std::cout << "Calibration complete." << std::endl;

    std::cout << "Saving betas to YAML." << std::endl;
    // saveToYAML(blimp_name, calibrator.beta);

    std::cout << "Showing live calibrated accelerometer samples." << std::endl;
    // Generate A and b from beta
    Eigen::MatrixXf A(3,3);
    A << beta(0), beta(1), beta(2),
         beta(1), beta(3), beta(4),
         beta(2), beta(4), beta(5);
    Eigen::VectorXf b(3);
    b << beta(6), beta(7), beta(8);

    std::cout << "True_beta: ";
    for(int i=0; i<9; i++) std::cout << beta_true(i) << ", ";
    std::cout << std::endl;

    // while(true){
    //     // Sample IMU
    //     float sample_floats[] = {0, 0, 0};
    //     func_sample_accelerometer_hardware(sample_floats);
    //     Eigen::VectorXf sample = Eigen::Map<Eigen::VectorXf>(sample_floats, 3);

    //     // Correct sample
    //     Eigen::VectorXf corrected_sample = A*sample - b;

    //     // Calculate norms
    //     float norm = sample.norm();
    //     float corrected_norm = corrected_sample.norm();

    //     // Print corrected samples
    //     std::cout << "[" << std::to_string(sample(0)) << ", " << std::to_string(sample(1)) << ", " <<std::to_string(sample(2)) << "], norm^2 = " << std::to_string(norm) << ",\t\t";
    //     std::cout << "[" << std::to_string(corrected_sample(0)) << ", " << std::to_string(corrected_sample(1)) << ", " <<std::to_string(corrected_sample(2)) << "], norm^2 = " << std::to_string(corrected_norm) << std::endl;

    //     // Delay
    //     func_delay_ms(500);
    // }

    return 0;
}