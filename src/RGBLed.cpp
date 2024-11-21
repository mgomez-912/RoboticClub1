#include "RGBLed.h"

const int pinLed = 48;           
const int pixels = 1;           // Number of LEDs (1 LED in this case)

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

const uint32_t colors2[] = {
    LedBuilt.Color(255,215,0),      // gold
    LedBuilt.Color(107,142,35),     // Olive drab
    LedBuilt.Color(47,79,79),       // dark slate gray
    LedBuilt.Color(127,255,212),    // aqua marine	#7FFFD4
    LedBuilt.Color(0,0,128),        // navy	#000080
    LedBuilt.Color(186,85,211),     // medium orchid	#BA55D3
    LedBuilt.Color(139,69,19),      // saddle brown	#8B4513
    LedBuilt.Color(255,127,80)         // coral	#FF7F50
};

// Task function that handles LED blinking
void RGBLed(void *pvParameters) {
    size_t numColors = sizeof(colors) / sizeof(colors[0]);
    size_t currentColor = 0;

    LedBuilt.begin();
    LedBuilt.show();  // Initialize all pixels to 'off'

    while (true) {
        if(channelValues[5]>1200 && channelValues[5]<1700){
            LedBuilt.fill(colors[currentColor]);  // Set all pixels to the current color
            LedBuilt.show();
        }

        if(channelValues[5]>1800 && channelValues[5]<2050){
            LedBuilt.fill(colors2[currentColor]);  // Set all pixels to the current color
            LedBuilt.show();
        }
        vTaskDelay(taskRGBLed.getIntervalms() / portTICK_PERIOD_MS);  // Delay for the blink time

        // Move to the next color
        currentColor = (currentColor + 1) % numColors;
    }
}
