#include "ColorSensor.h"

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
    if (!tcs.begin()) {
        Serial.println("Color sensor not found. Halting task.");
        vTaskDelete(NULL); // Stop task if initialization fails
        return;
    }

    while (true) {
        uint16_t r, g, b, c;
        tcs.getRawData(&r, &g, &b, &c);

        if (c > 1000 && (b - r - g) > 600) {
            detectedColor = "blue";
            Serial.println("BLUE DETECTED");
        } 
        else if (c > 700 && (r - b - g) > 300) {
            detectedColor = "red";
            Serial.println("RED DETECTED");
        } else {
            detectedColor = "unknown";
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
