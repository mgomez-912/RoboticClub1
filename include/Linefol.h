#ifndef TASKLINEFOLLOW_H
#define TASKLINEFOLLOW_H

#pragma once

#include <Arduino.h>
#include "GlobalVariables.h"

#include <QuickPID.h>
#include "esp32-hal-timer.h"


extern HardwareSerial SerialLine; // UART1: RX=13, TX=14
const int lineRX = 16;
const int lineTX = 17;

// Data variables
extern uint8_t Patrol_data[3];
extern int  deltime;
extern int position;

///////////////////////////////////
// PID Parameters
const float Kp = 0.00415, Ki = 0.00005, Kd = 0.00190;      //0.005, 0.004.... 0.001, 0.0009
const int outputLimit = 250;  // Max rotation adjustment

extern QuickPID linePID;
extern float linePosition;    // Current line position (0-7000)
extern float rotationOutput;  // PID output for motor control
extern const int outputLimit;

//Functions
void LineFollow(void *pvParameters);
void processSensorData();
int calculatePosition(uint8_t status);
void sendRequest();
void initPIDController();
void actionsPID(int status);

#endif