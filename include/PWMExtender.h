#ifndef PWM_EXTENDER_H
#define PWM_EXTENDER_H

#include <Arduino.h>
#include "GlobalVariables.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>

// Create PCA9685 object
extern Adafruit_PWMServoDriver pwm;

// Define servo parameters
#define SERVO_CHANNEL 0     // Channel on PCA9685 (0-15)
#define SERVO_MIN 100       // Minimum pulse length count
#define SERVO_MAX 500      // Maximum pulse length count
#define SERVO_FREQ 50       // Servo frequency (50 Hz)

// Define custom I2C pins for ESP32
#define SDA_PIN 20          // Data pin (SDA)
#define SCL_PIN 21          // Clock pin (SCL)

////////////////////////////////////////////
// Configuration
#define MOTOR_CHANNEL 4          // PCA9685 channel for motor
#define SAFETY_CHANNEL 4         // Channel for safety switch
#define CONTROL_CHANNEL 2        // Channel for speed control
#define RAMP_STEP 5              // Speed change per cycle (adjust as needed)

// Function declarations
void PWMExtender(void *pvParameters); // Task function
void initializeServo(int);      // Initialize PCA9685
void sweepServo(int);          // Sweep servo back and forth
void setServoAngle(uint8_t channel, uint8_t angle); // Set specific angle

int pulseWidth(int speed);

#endif