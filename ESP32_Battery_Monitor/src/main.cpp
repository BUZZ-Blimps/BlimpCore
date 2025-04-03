#include <Arduino.h>

// Pin Definitions
const int SENSOR_PIN = A2; // Analog pin for voltage measurement
const int BUZZER_PIN = D0; // Digital pin for buzzer control

// Voltage Reference and ADC Resolution
const float ADC_RESOLUTION = 4095.0; // 12-bit ADC
const float VREF = 3.3;              // Reference voltage
const float VOLTAGE_DIVIDER_RATIO = 5.0; // Voltage divider ratio
const float OFFSET = 0.24;           // Calibration offset

// Voltage Thresholds and Hysteresis
const float LOW_VOLTAGE_THRESHOLD = 7.35;       // Voltage threshold to start beeping
const float CRITICAL_VOLTAGE_THRESHOLD = 7.0;   // Voltage threshold for rapid beeping
const float HYSTERESIS_MARGIN = 0.1;            // Voltage margin to stop beeping
const int REQUIRED_LOW_VOLTAGE_COUNT = 15;      // Number of consecutive low readings required to activate buzzer
const float BATTERY_DISCONNECTED_THRESHOLD = 0.5; // Voltage threshold indicating battery is disconnected

// Timing Intervals
const unsigned long VOLTAGE_INTERVAL = 200; // Interval for voltage reading in milliseconds (5 Hz)
unsigned long previousVoltageMillis = 0;
unsigned long previousBuzzerMillis = 0;
unsigned long buzzerInterval = 1000; // Initial interval for buzzer beeping

// Buzzer State Variables
bool buzzerState = false;  // Tracks the current state of the buzzer
bool buzzerEnabled = false; // Indicates if the buzzer should be active
int lowVoltageCount = 0;    // Counter for consecutive low voltage readings

// Kalman Filter Variables
float kalman_gain = 0.1;
float estimate = 0;
float estimate_error = 1;
float process_noise = 0.01;
float measurement_noise = 0.1;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // Set ADC resolution to 12 bits
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // Ensure buzzer is off initially
}

float kalman_filter(float measurement) {
  // Prediction step
  float prediction = estimate;
  estimate_error += process_noise;

  // Update step
  kalman_gain = estimate_error / (estimate_error + measurement_noise);
  estimate = prediction + kalman_gain * (measurement - prediction);
  estimate_error *= (1 - kalman_gain);

  return estimate;
}

void loop() {
  unsigned long currentMillis = millis();

  // Voltage Reading and Serial Output
  if (currentMillis - previousVoltageMillis >= VOLTAGE_INTERVAL) {
    previousVoltageMillis = currentMillis;

    int rawValue = analogRead(SENSOR_PIN);
    float voltage = ((rawValue / ADC_RESOLUTION) * VREF * VOLTAGE_DIVIDER_RATIO) + OFFSET;

    // Apply Kalman filter
    float filtered_voltage = kalman_filter(voltage);

    // Check for battery disconnection
    if (filtered_voltage < BATTERY_DISCONNECTED_THRESHOLD) {
      buzzerEnabled = false;
      digitalWrite(BUZZER_PIN, LOW); // Turn off buzzer
    } else {
      Serial.println(filtered_voltage, 2);

      // Hysteresis Logic for Buzzer Activation
      if (!buzzerEnabled) {
        if (filtered_voltage <= LOW_VOLTAGE_THRESHOLD) {
          lowVoltageCount++;
          if (lowVoltageCount >= REQUIRED_LOW_VOLTAGE_COUNT) {
            buzzerEnabled = true;
            lowVoltageCount = 0; // Reset counter
          }
        } else {
          lowVoltageCount = 0; // Reset counter if voltage rises above threshold
        }
      } else {
        if (filtered_voltage > LOW_VOLTAGE_THRESHOLD + HYSTERESIS_MARGIN) {
          buzzerEnabled = false;
          digitalWrite(BUZZER_PIN, LOW); // Turn off buzzer
        }
      }

      // Determine Buzzer Interval Based on Voltage Levels
      if (buzzerEnabled) {
        if (filtered_voltage <= CRITICAL_VOLTAGE_THRESHOLD) {
          buzzerInterval = 100; // Fast beeping at critical voltage
        } else if (filtered_voltage <= LOW_VOLTAGE_THRESHOLD) {
          // Define discrete buzzer intervals
          if (filtered_voltage <= CRITICAL_VOLTAGE_THRESHOLD + 0.1) {
            buzzerInterval = 200;
          } else if (filtered_voltage <= CRITICAL_VOLTAGE_THRESHOLD + 0.2) {
            buzzerInterval = 400;
          } else if (filtered_voltage <= CRITICAL_VOLTAGE_THRESHOLD + 0.3) {
            buzzerInterval = 600;
          } else if (filtered_voltage <= CRITICAL_VOLTAGE_THRESHOLD + 0.4) {
            buzzerInterval = 800;
          } else {
            buzzerInterval = 1000;
          }
        }
      }
    }
  }

  // Buzzer Control
  if (buzzerEnabled && (currentMillis - previousBuzzerMillis >= buzzerInterval)) {
    previousBuzzerMillis = currentMillis;
    buzzerState = !buzzerState; // Toggle buzzer state
    digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW); // Active-high buzzer
  }
}
