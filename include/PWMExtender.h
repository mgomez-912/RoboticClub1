#ifndef PWM_EXTENDER_H
#define PWM_EXTENDER_H

#include <Arduino.h>
#include "GlobalVariables.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>

// called this way, it uses the default address 0x40
const int MIN_PULSE_WIDTH = 650;
const int MAX_PULSE_WIDTH = 2350;
const int DEFAULT_PULSE_WIDTH = 1500;
const int FREQUENCY = 50;


void PWMExtender(void *pvParameters);
int pulseWidth(int angle);

#endif