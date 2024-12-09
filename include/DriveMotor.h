#ifndef DRIVE_MOTOR_H
#define DRIVE_MOTOR_H

#include <Arduino.h>
#include "GlobalVariables.h"

// Motor control pins
const int M1_Forward = 4;
const int M1_Reverse = 5;
const int M2_Forward = 6;
const int M2_Reverse = 7;

const int motorProt = 500;  // Default protection time for motor direction change
const int speedlim = 150;   // Maximum input for the motor driver (0 - 255)

// Channel values
const int minres = 1450;
const int maxres = 1550;

// Ramp-up configuration
const int rampStep = 5;         // Step size for speed increase
const int rampDelay = 5;        // Delay in milliseconds for each ramp step


void MotorDriving(void *pvParameters);          // Main task

// Function prototypes
void updateState(int advSignal, int turnSignal);
void executeState();
void stopMotors();
void executeActionImmediately();
void setMotorDirection(bool dirM1, int targetSpeedM1, bool dirM2, int targetSpeedM2);
void rampSpeed(int &currentSpeed, int targetSpeed, int controlPinForward, int controlPinReverse);
void controlForward();
void controlReverse();

#endif