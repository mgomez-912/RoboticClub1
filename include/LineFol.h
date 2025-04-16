#ifndef TASKLINEFOLLOW_H
#define TASKLINEFOLLOW_H

#include <Arduino.h>
#include "GlobalVariables.h"
#include <Wire.h>

extern HardwareSerial SerialLine; // UART1: RX=13, TX=14
const int lineRX = 16;
const int lineTX = 17;

//Functions
void LineFollow(void *pvParameters);

#endif