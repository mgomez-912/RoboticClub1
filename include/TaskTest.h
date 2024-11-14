#ifndef TASK_TEST_H
#define TASK_TEST_H

#include <Arduino.h>
#include "GlobalVariables.h"

const int test = LED_BUILTIN;      

void taskTest(void *pvParameters);  // Declaration of task function

#endif