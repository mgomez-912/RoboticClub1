#include "RGBLed.h"

#define pinLed 48           
#define pixels  1     // Number of LEDs (1 LED in this case)
// #define blinkTime  500   // Delay time in milliseconds

Adafruit_NeoPixel LedBuilt(1, pinLed, NEO_GRB + NEO_KHZ800);

// Define the colors to cycle through (Red, Green, Blue, Yellow, Magenta, Cyan, White, Off)
const uint32_t colors[] = {
    LedBuilt.Color(255, 0, 0),     // Red
    LedBuilt.Color(0, 255, 0),     // Green
    LedBuilt.Color(0, 0, 255),     // Blue
    LedBuilt.Color(255, 255, 0),   // Yellow
    LedBuilt.Color(255, 0, 255),   // Magenta
    LedBuilt.Color(0, 255, 255),   // Cyan
    LedBuilt.Color(255, 255, 255), // White
    LedBuilt.Color(0, 0, 0)        // Off
};

// Task function that handles LED blinking
void RGBLed(void *pvParameters) {
    size_t numColors = sizeof(colors) / sizeof(colors[0]);
    size_t currentColor = 0;

    LedBuilt.begin();
    LedBuilt.show();  // Initialize all pixels to 'off'

    while (true) {
        LedBuilt.fill(colors[currentColor]);  // Set all pixels to the current color
        LedBuilt.show();
        // vTaskDelay(pdMS_TO_TICKS(blinkTime));
        vTaskDelay(taskRGBLed.getIntervalms() / portTICK_PERIOD_MS);  // Delay for the blink time

        // Move to the next color
        currentColor = (currentColor + 1) % numColors;
    }
}
