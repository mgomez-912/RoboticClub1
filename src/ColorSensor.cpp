#include "ColorSensor.h"

#include <Wire.h>
#include <Arduino.h>

// Define the TCS34725 color sensor object
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
    TCS34725_INTEGRATIONTIME_50MS,
    TCS34725_GAIN_4X
);

// Define the detected color string
String detectedColor = "";

// Color detection FreeRTOS task
void ColorSensor(void *pvParameters) {
    Serial.println("ColorTask started");

    // Initialize the sensor
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!tcs.begin()) {
        Serial.println("Color sensor not found. Halting task.");
        vTaskDelete(NULL); // Stop task if initialization fails
        return;
    }

    while (true) {
        uint16_t r, g, b, c;
        tcs.getRawData(&r, &g, &b, &c);
        Serial.printf("R:%d G:%d B:%d C:%d\n", r, g, b, c);
        if (c > 400 && ((float)b/c) > 0.5) {
            detectedColor = "blue";
            Serial.println("BLUE DETECTED");
        } 
        else if (c > 350 && ((float)r/c) > 0.5) {
            detectedColor = "red";
            Serial.println("RED DETECTED");
        } else {
            detectedColor = "unknown";
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
