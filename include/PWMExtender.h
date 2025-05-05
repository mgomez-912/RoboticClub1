#ifndef PWM_EXTENDER_H
#define PWM_EXTENDER_H

#include <Arduino.h>
#include "GlobalVariables.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create PCA9685 object
extern Adafruit_PWMServoDriver pwm;

// Define servo parameters
#define SERVO_CHANNEL 0     // Channel on PCA9685 (0-15)
#define SERVO_MIN 100       // Minimum pulse length count
#define SERVO_MAX 500      // Maximum pulse length count
#define SERVO_FREQ 50       // Servo frequency (50 Hz)

// Define custom I2C pins for ESP32
#define SDA_PIN 40          // Data pin (SDA)
#define SCL_PIN 41          // Clock pin (SCL)

// Function declarations
void PWMExtender(void *pvParameters); // Task function
void initializeServo(int);      // Initialize PCA9685
void sweepServo(int);          // Sweep servo back and forth
void setServoAngle(uint8_t channel, uint8_t angle); // Set specific angle

#endif