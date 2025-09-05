#include <SPI.h>  // include the new SPI library:
#include "Arduino.h"

// using two incompatible SPI devices, A and B
const int slave_LSM6DSL_Pin = 10;
const int slave_LIS3MDL_Pin = 9;

// set up the speed, mode and endianness of each device
SPISettings settingsLSM(10000000, MSBFIRST, SPI_MODE3); 
SPISettings settingsB(16000000, LSBFIRST, SPI_MODE3); 

void setup() {
  // set the Slave Select Pins as outputs:
  pinMode (slave_LSM6DSL_Pin, OUTPUT);
  pinMode (slaveBPin, OUTPUT);
  // initialize SPI:
  SPI.begin(); 
}

uint8_t stat, val1, val2, result;

void loop() {
  // read three bytes from device A
  SPI.beginTransaction(settingsLSM);
  digitalWrite (slave_LSM6DSL_Pin, LOW);
  // reading only, so data sent does not matter
  stat = SPI.transfer(0);
  val1 = SPI.transfer(0);
  val2 = SPI.transfer(0);
  digitalWrite (slaveAPin, HIGH);
  SPI.endTransaction();
  // if stat is 1 or 2, send val1 or val2 else zero
  if (stat == 1) { 
   result = val1;
  } else if (stat == 2) { 
   result = val2;
  } else {
   result = 0;
  }
  // send result to device B
  SPI.beginTransaction(settingsB);
  digitalWrite (slaveBPin, LOW);
  SPI.transfer(result);
  digitalWrite (slaveBPin, HIGH);
  SPI.endTransaction();
}