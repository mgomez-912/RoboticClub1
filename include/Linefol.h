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
extern int deltime;
extern int position;

extern int lost_count;                  //Counter for the cycles when can see any line
extern int inter_count;                 //Counter for the intersection crossed

///////////////////////////////////
// PID Parameters
const float Kp = 0.00425, Ki = 0.0002, Kd = 0.0023; //kp=0.0038-44, ki=0.0001-2 , kd=0.0018-30 
// const float Kp_lost = 0.008, Ki_lost = 0.00000, Kd_lost = 0.0025;

const int outputLimit = 250;  // Max rotation adjustment

extern QuickPID linePID;
extern float linePosition;    // Current line position (0-7000)
extern float rotationOutput;  // PID output for motor control
extern const int outputLimit;

//////////////////////////////////////
enum IntersectionPhase { NONE, SLIDE, ROTATE, DONE };

//Functions
void LineFollow(void *pvParameters);
void processSensorData();
int calculatePosition(uint8_t status);
void sendRequest();
void initPIDController();
void actionsPID(int status);


#endif