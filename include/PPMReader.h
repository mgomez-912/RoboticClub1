#ifndef TASK_PPM_READ_H
#define TASK_PPM_READ_H

#include <Arduino.h>
#include "GlobalVariables.h"

void RXRread(void *pvParameters);
void IRAM_ATTR ppmInterruptHandler();

#endif