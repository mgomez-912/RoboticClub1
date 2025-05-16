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
const int SDA_PIN = 40;          // Data pin (SDA)
const int SCL_PIN = 41;          // Clock pin (SCL)

/////////////////////////////////////////
enum ArmTarget {
    NONE,
    PICKUP_1,
    PICKUP_2,
    DELIVERY,
    STORAGE
};

const int openClaw = 160;
const int closedClaw = 100;
const int front = 65; 
const int back = 180;
const int pauseMov = 150;

const int deadBHigh = 1750;
const int deadBLow = 1250;


// Function declarations
void PWMExtender(void *pvParameters); // Task function
void initializeServo(int);      // Initialize PCA9685

void setServoAngle(uint8_t channel, uint8_t angle); // Set specific angle

void armPos(int, int, int, int, int, int);
void inputHandle();

ArmTarget getArmTargetFromSwitches();
void moveArmSafely(int to[5]);
void processArmPosition();

#endif