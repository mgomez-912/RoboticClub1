#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// Forward declaration of the FreeRTOS task
void ColorSensor(void *pvParameters);

// Declare the color sensor object (extern if defined elsewhere)
extern Adafruit_TCS34725 tcs;

// Detected color string (shared across files)
extern String detectedColor;

// Define custom I2C pins for ESP32
#define SDA_PIN 40          // Data pin (SDA)
#define SCL_PIN 41          // Clock pin (SCL)

#endif // COLOR_SENSOR_H
