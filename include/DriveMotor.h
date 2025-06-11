#ifndef DRIVE_MOTOR_H
#define DRIVE_MOTOR_H

#include <Arduino.h>
#include "GlobalVariables.h"
#include <driver/timer.h>

//version bjgala
const int M1_Forward = 35;
const int M1_Reverse = 36;
const int M3_Forward = 37;
const int M3_Reverse = 38;

const int M2_Forward = 4;
const int M2_Reverse = 5;
const int M4_Forward = 6;
const int M4_Reverse = 7;

// Motor State Structure
struct MotorState {
    bool currentDir;    // Current direction (true = forward)
    int currentSpeed;   // Current PWM value
    int targetSpeed;    // Desired PWM value
    int forwardPin;
    int reversePin;
};
extern MotorState motors[4];

const int motorProt = 50;  // Default protection time for motor direction change
const int speedlim = 45;   // Maximum input for the motor driver (0 - 255)

// Channel values
const int minres = 980;
const int maxres = 2020;

// Ramp-up configuration
const int rampStep = 5;         // Step size for speed increase
const int rampDelay = 5;        // Delay in milliseconds for each ramp step

const int deadzone = 50;        // Adjusted deadzone
const int stopThreshold = 20;   // Immediate stop threshold

// Function prototypes
void MotorDriving(void *pvParameters);          // Main task
// int scaleChannel(int channelValue);
int scaleChannel(int channelValue, bool invert);
void stopMotors();
void updateMotor(MotorState& m);  // Keep reference symbol (&) attached to type

// int calculateRampStep(int prevTarget, int currentTarget);
void calculateMotors(int throttle, int strafe, int rotation);

#endif