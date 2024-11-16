#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <math.h>
#include <fstream>
#include <iomanip>

#include "OPI_IMU.hpp"
#include "AccelerometerCalibrator.hpp"

OPI_IMU imu;
AccelerometerCalibrator calibrator;

void func_sample_accelerometer_hardware(float* data){
    imu.IMU_read();
    data[0] = imu.AccXraw;
    data[1] = imu.AccYraw;
    data[2] = imu.AccZraw;
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
    calibrator.compute_calibration();
    std::cout << "Calibration complete." << std::endl;

    std::cout << "Saving betas to YAML." << std::endl;
    saveToYAML(blimp_name, calibrator.beta);

    std::cout << "Showing live calibrated accelerometer samples." << std::endl;
    while(true){
        // Sample IMU
        float sample_floats[] = {0, 0, 0};
        func_sample_accelerometer_hardware(sample_floats);
        Eigen::VectorXf sample = Eigen::Map<Eigen::VectorXf>(sample_floats, 3);

        // Correct sample
        Eigen::VectorXf corrected_sample = (sample - calibrator.beta.segment(0,3)).cwiseProduct(calibrator.beta.segment(3,3));

        // Calculate norms
        float norm = sample.norm();
        float corrected_norm = corrected_sample.norm();

        // Print corrected samples
        std::cout << "[" << std::to_string(sample(0)) << ", " << std::to_string(sample(1)) << ", " <<std::to_string(sample(2)) << "], norm^2 = " << std::to_string(norm) << ",\t\t";
        std::cout << "[" << std::to_string(corrected_sample(0)) << ", " << std::to_string(corrected_sample(1)) << ", " <<std::to_string(corrected_sample(2)) << "], norm^2 = " << std::to_string(corrected_norm) << std::endl;

        // Delay
        func_delay_ms(500);
    }

    return 0;
}