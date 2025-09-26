#include <SPI.h>  // include the new SPI library:
#include "Arduino.h"
#include "BerryIMU_v3.hpp"

// using two incompatible SPI devices, A and B
const int slave_LSM6DSL_Pin = 10;
const int slave_LIS3MDL_Pin = 9;

float time1;
float time2;

int count = 10000;


// set up the speed, mode and endianness of each device
// SPISettings settingsLSM(1000000, MSBFIRST, SPI_MODE3); 
// SPISettings settingsB(16000000, LSBFIRST, SPI_MODE3); 
BerryIMU_v3 imu;

void setup() {
  // set the Slave Select Pins as outputs:
  Serial.begin(115200);
  pinMode (slave_LSM6DSL_Pin, OUTPUT);
  digitalWrite (slave_LSM6DSL_Pin, HIGH);
//   pinMode (slaveBPin, OUTPUT);
  // initialize SPI:
  SPI.begin(); 
  imu.BerryIMU_v3_Setup();
//   SPI1.beginTransaction(settingsLSM);
}

uint16_t stat, val1, val2, result;

void loop() {

   imu.IMU_read();
    if (count == 10000){
        time1 = millis();
    }
    if (count == 0){
        time2= millis();
    }
    if (count <= 0){
        Serial.println(10000/((time2-time1)/1000));
    }

    // Print Statements
    Serial.print("AccXraw: ");
    Serial.println(imu.AccXraw);

    Serial.print("AccYraw: ");
    Serial.println(imu.AccYraw);

    // Print Statements
    Serial.print("AccZraw: ");
    Serial.println(imu.AccZraw);

    Serial.print("gyr_rateXraw: ");
    Serial.println(imu.gyr_rateXraw);

    // Print Statements
    Serial.print("gyr_rateYraw: ");
    Serial.println(imu.gyr_rateYraw);

    Serial.print("gyr_rateZraw: ");
    Serial.println(imu.gyr_rateZraw);

    // Print Statements
    Serial.print("MagXraw: ");
    Serial.println(imu.MagXraw);

    Serial.print("MagYraw: ");
    Serial.println(imu.MagYraw);

    Serial.print("MagZraw: ");
    Serial.println(imu.MagZraw);
    count--;
}

