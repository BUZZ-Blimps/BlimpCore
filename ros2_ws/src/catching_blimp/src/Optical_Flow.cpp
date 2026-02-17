/*
  Optical_Flow.cpp - Library that will output the flow rate and surface quality content
*/
#define HWSERIAL Serial3
#include <math.h>
#include "Optical_Flow.hpp"
#define FOOTER 0xAA
#define HEADER 0xFE
#include <iterator>

Optical_Flow::Optical_Flow() {
  HWSERIAL.begin(19200);
  while (!HWSERIAL) {
    delay(100);
  }
}

void Optical_Flow::read_buffer() {
  while (HWSERIAL.available()) {
    uint8_t c = HWSERIAL.read();
    buffer.push_back(c);
  }
}

void Optical_Flow::update_flow(float roll_rate, float pitch_rate, float Z_distance_m) {
  bool foundPacket = false;
  if (!buffer.empty()) {
    std::vector<uint8_t>::iterator it = buffer.end();
    while (it != buffer.begin()) {
      if (*it == FOOTER) {
        int distToHead = std::distance(buffer.begin(), it);
        if (distToHead >= 8) {
          std::vector<uint8_t>::iterator headerCheck = it - 8;
          if (*(it - 8) == HEADER) {
            if (*(headerCheck + 1) == 4) {
              foundPacket = true;

              uint8_t x_motion_HB = *(headerCheck + 2);
              uint8_t x_motion_LB = *(headerCheck + 3);
              uint8_t y_motion_HB = *(headerCheck + 4);
              uint8_t y_motion_LB = *(headerCheck + 5);
              uint8_t checksum = *(headerCheck + 6);
              surface_quality = *(headerCheck + 7);

              x_motion = (((uint16_t)x_motion_LB << 8) | x_motion_HB);
              y_motion = (((uint16_t)y_motion_LB << 8) | y_motion_HB);

              float FOV = 42;
              float sensor_res = 1225;
              float deg_rad = (FOV/2)*0.0174533;
              float scalar_alt = .005;

              float x_motion_scaled = (((x_motion * Z_distance_m) / (sensor_res * scalar_alt)) * 2 * tan(deg_rad));
              float y_motion_scaled = ((y_motion * Z_distance_m) / (sensor_res * scalar_alt)) * 2 * tan(deg_rad);

              float scalar_comp = .00035;
              float x_motion_change = (pitch_rate*sensor_res*scalar_comp)/FOV;
              float y_motion_change = (roll_rate*sensor_res*scalar_comp)/FOV;

              x_motion_comp = x_motion_scaled - x_motion_change;
              y_motion_comp = y_motion_scaled - y_motion_change;

              break;
            }
          }
        }
      }
      it--;
    }
    if (foundPacket || buffer.size() >= 100) {
      buffer.clear();
    }
  }
}
