#ifndef TASK_TEST_H
#define TASK_TEST_H

#include <Arduino.h>
#include "GlobalVariables.h"

const int test = LED_BUILTIN;
const int M1F = 35;  
const int M1R = 36;
const int M2F = 37;
const int M2R = 38;    

void taskTest(void *pvParameters);  // Declaration of task function

void stopMotors();

#endif