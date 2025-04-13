#ifndef TASKSERVOPCA_H
#define TASKSERVOPCA_H

#include <Arduino.h>
#include "GlobalVariables.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create PCA9685 object
extern Adafruit_PWMServoDriver pwm;

// Define servo parameters
#define SERVO_CHANNEL 0     // Channel on PCA9685 (0-15)
#define SERVO_MIN 150       // Minimum pulse length count
#define SERVO_MAX 600       // Maximum pulse length count
#define SERVO_FREQ 50       // Servo frequency (50 Hz)

// Define custom I2C pins for ESP32
#define SDA_PIN 20          // Data pin (SDA)
#define SCL_PIN 21          // Clock pin (SCL)

// Function declarations
void ServoPCA(void *pvParameters); // Task function
void initializeServo();      // Initialize PCA9685
void sweepServo();          // Sweep servo back and forth
void setServoAngle(uint8_t channel, uint8_t angle); // Set specific angle
void setServoPulse(uint8_t n, double pulse); // Set pulse length

#endif