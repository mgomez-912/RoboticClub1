#ifndef TASKLINEFOLLOW_H
#define TASKLINEFOLLOW_H

#include <Arduino.h>
#include "GlobalVariables.h"


extern HardwareSerial SerialLine; // UART1: RX=13, TX=14
const int lineRX = 16;
const int lineTX = 17;

// Data variables
extern uint8_t Patrol_data[3];
extern int  deltime;

//Functions
void LineFollow(void *pvParameters);
void processSensorData();
int calculatePosition(uint8_t status);
void sendRequest() ;

#endif