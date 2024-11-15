#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <math.h>

#include "OPI_IMU.hpp"
#include "AccelerometerCalibrator.hpp"

OPI_IMU imu;
AccelerometerCalibrator calibrator;

void sample_accelerometer_hardware(float* data){
    imu.IMU_read();
    data[0] = imu.AccXraw;
    data[1] = imu.AccYraw;
    data[2] = imu.AccZraw;
}

void func_sample_accelerometer_hardware_long(long* data){
    float data_float[] = {0, 0, 0};
    sample_accelerometer_hardware(data_float);

    for(int i=0; i<3; i++) data[i] = *((long*)(&(data_float[i])));
    // std::cout << "pre-sample: " << std::to_string(data_float[0]) << ", " + std::to_string(data[0]) << std::endl;
}

void func_print(std::string message){
    std::cout << message;
}

void func_delay_ms(int delay_length_ms){
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_length_ms));
}

int main(){
    imu.OPI_IMU_Setup();
    calibrator.init(func_sample_accelerometer_hardware_long, func_print, func_delay_ms);

    int num_samples = 10;

    std::cout << "Beginning calibration with " << std::to_string(num_samples) << " samples." << std::endl;
    std::cout << "To take a new sample, put the IMU in a new rotation and press ENTER when it is still. DO NOT MOVE IT." << std::endl;

    for(int i=0; i<num_samples; i++){
        std::cout << "Sample " << std::to_string(i) << ". Press ENTER.";
        std::cin.ignore();
        calibrator.take_sample();
    }

    std::cout << "All samples taken. Beginning calibration." << std::endl;
    calibrator.compute_calibration();

    std::cout << "Calibration complete. Showing live calibrated accelerometer samples." << std::endl;
    while(true){
        // Sample IMU
        float sample[] = {0, 0, 0};
        sample_accelerometer_hardware(sample);

        // Correct sample
        float corrected_sample[] = {0, 0, 0};
        for(int i=0; i<3; i++) corrected_sample[i] = (sample[i] - calibrator.beta[i]) * calibrator.beta[i+3];

        // Print corrected samples
        float norm = sqrt(pow(corrected_sample[0],2) + pow(corrected_sample[1],2) + pow(corrected_sample[2],2));
        std::cout << "[" << std::to_string(corrected_sample[0]) << ", " << std::to_string(corrected_sample[1]) << ", " <<std::to_string(corrected_sample[2]) << "], norm^2 = " << std::to_string(norm) << std::endl;

        // Delay
        func_delay_ms(500);
    }

    return 0;
}