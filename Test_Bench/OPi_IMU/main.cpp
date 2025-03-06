#include "OPI_IMU.h"
#include <string.h>
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

int main(){
    OPI_IMU imu;
    imu.OPI_IMU_Setup();
    while(1){
        imu.IMU_read();
        imu.IMU_ROTATION(0); // Rotate IMU
        // printf("ROTATION 0 DEGREES\n");
        // printf("========================\n");
        // printf("AccYraw: %f\n", imu.AccYraw);
        // printf("AccXraw: %f\n", imu.AccXraw);
        // printf("AccZraw: %f\n\n", imu.AccZraw);

        printf("MagYraw: %f\n", imu.MagYraw);
        printf("MagXraw: %f\n", imu.MagXraw);
        printf("MagZraw: %f\n", imu.MagZraw);
        printf("Compass: %f\n\n", std::atan2(imu.MagYraw, imu.MagXraw)*180/M_PI);

        // printf("gyr_rateYraw: %f\n", imu.gyr_rateYraw);
        // printf("gyr_rateXraw: %f\n", imu.gyr_rateXraw);
        // printf("gyr_rateZraw: %f\n\n", imu.gyr_rateZraw);

        // printf("tempRaw: %f\n", imu.tempRaw);
        // printf("pressRaw: %f\n\n\n\n", imu.pressRaw);
        delay(100);

        // imu.IMU_read();
        // imu.IMU_ROTATION(90); // Rotate IMU
        // printf("ROTATION 90 DEGREES\n");
        // printf("========================\n");
        // printf("AccYraw: %f\n", imu.AccYraw);
        // printf("AccXraw: %f\n", imu.AccXraw);
        // printf("AccZraw: %f\n\n", imu.AccZraw);

        // printf("MagYraw: %f\n", imu.MagYraw);
        // printf("MagXraw: %f\n", imu.MagXraw);
        // printf("MagZraw: %f\n\n", imu.MagZraw);

        // printf("gyr_rateYraw: %f\n", imu.gyr_rateYraw);
        // printf("gyr_rateXraw: %f\n", imu.gyr_rateXraw);
        // printf("gyr_rateZraw: %f\n\n", imu.gyr_rateZraw);

        // printf("tempRaw: %f\n", imu.tempRaw);
        // printf("pressRaw: %f\n\n\n\n", imu.pressRaw);
        // delay(100);
    }
    return(1);
}