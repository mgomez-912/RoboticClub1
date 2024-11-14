#ifndef RGBLEDTASK_H
#define RGBLEDTASK_H

#include <Arduino.h>
#include "GlobalVariables.h"
#include <Adafruit_NeoPixel.h>


#define pinLed 48           
#define pixels  1     // Number of LEDs (1 LED in this case)
// #define blinkTime  500   // Delay time in milliseconds

void RGBLed(void *pvParameters);

#endif