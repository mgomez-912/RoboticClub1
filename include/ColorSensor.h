#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <Adafruit_TCS34725.h>

// Forward declaration of the FreeRTOS task
void ColorSensor(void *pvParameters);

// Declare the color sensor object (extern if defined elsewhere)
extern Adafruit_TCS34725 tcs;

// Detected color string (shared across files)
extern String detectedColor;

#ifndef SDA_PIN
  #define SDA_PIN 40   // Default if not defined elsewhere
#endif

#ifndef SCL_PIN
  #define SCL_PIN 41   // Default if not defined elsewhere
#endif

#endif // COLOR_SENSOR_H
