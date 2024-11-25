// #include "RGBLed.h"

// const int pinLed = 40;           
// const int pixels = 32;           // Number of LEDs (1 LED in this case)

// Adafruit_NeoPixel LedBuilt(1, pinLed, NEO_GRB + NEO_KHZ800);

// // Define the colors to cycle through (Red, Green, Blue, Yellow, Magenta, Cyan, White, Off)
// const uint32_t colors[] = {
//     LedBuilt.Color(255, 0, 0),     // Red
//     LedBuilt.Color(0, 255, 0),     // Green
//     LedBuilt.Color(0, 0, 255),     // Blue
//     LedBuilt.Color(255, 255, 0),   // Yellow
//     LedBuilt.Color(255, 0, 255),   // Magenta
//     LedBuilt.Color(0, 255, 255),   // Cyan
//     LedBuilt.Color(255, 255, 255), // White
//     LedBuilt.Color(0, 0, 0)        // Off
// };

// const uint32_t colors2[] = {
//     LedBuilt.Color(255,215,0),      // gold
//     LedBuilt.Color(107,142,35),     // Olive drab
//     LedBuilt.Color(47,79,79),       // dark slate gray
//     LedBuilt.Color(127,255,212),    // aqua marine	#7FFFD4
//     LedBuilt.Color(0,0,128),        // navy	#000080
//     LedBuilt.Color(186,85,211),     // medium orchid	#BA55D3
//     LedBuilt.Color(139,69,19),      // saddle brown	#8B4513
//     LedBuilt.Color(255,127,80)         // coral	#FF7F50
// };

// // Task function that handles LED blinking
// void RGBLed(void *pvParameters) {
//     size_t numColors = sizeof(colors) / sizeof(colors[0]);
//     size_t currentColor = 0;

//     LedBuilt.begin();
//     LedBuilt.show();  // Initialize all pixels to 'off'

//     while (true) {
//         if(channelValues[5]>1200 && channelValues[5]<1700){
//             LedBuilt.fill(colors[currentColor]);  // Set all pixels to the current color
//             LedBuilt.show();
//         }else if(channelValues[5]>1800 && channelValues[5]<2050){
//             LedBuilt.fill(colors2[currentColor]);  // Set all pixels to the current color
//             LedBuilt.show();
//         } else {
//             LedBuilt.fill((0,0,0));
//             LedBuilt.show();
//         }

//         vTaskDelay(taskRGBLed.getIntervalms() / portTICK_PERIOD_MS);  // Delay for the blink time

//         // Move to the next color
//         currentColor = (currentColor + 1) % numColors;
//     }
// }


//Rainbow pattern
#include "RGBLed.h"

const int pinLed = 41;           // GPIO pin connected to the LED strip 41 or 42
const int pixels = 32;           // Number of LEDs in the WS2815 strip

Adafruit_NeoPixel LedStrip(pixels, pinLed, NEO_GRB + NEO_KHZ800);

// Task function to create a fast-moving rainbow pattern
void RGBLed(void *pvParameters) {
    LedStrip.begin();
    LedStrip.show();  // Initialize all pixels to 'off'

    int colorOffset = 0;  // Offset for shifting colors along the strip
    int frame = 0;  // Animation frame counter
    float brightness = 0.5;

    while (true) {
        if(channelValues[5]>1200 && channelValues[5]<1700){
            // Update the colors on the strip based on the offset
            for (int i = 0; i < LedStrip.numPixels(); i++) {
                // Calculate the color for this pixel
                int hue = (colorOffset + (i * 360 / LedStrip.numPixels())) % 360;
                uint32_t color = LedStrip.gamma32(LedStrip.ColorHSV(hue * 182));  // Convert hue to color
                LedStrip.setPixelColor(i, color);
                LedStrip.show();

                // Increment the offset more aggressively for faster movement
                colorOffset = (colorOffset + 10) % 360;}
            }else if(channelValues[5]>1800 && channelValues[5]<2050){
                // Update the colors on the strip for the current frame
                for (int i = 0; i < LedStrip.numPixels(); i++) {
                    // Create a gradient effect across the strip
                    int hue = (frame + (i * 360 / LedStrip.numPixels())) % 360;  // Color gradient
                    uint32_t color = LedStrip.ColorHSV(hue * 182);  // Convert hue to color

                    // Apply brightness adjustment
                    uint8_t red = (uint8_t)(((color >> 16) & 0xFF) * brightness);
                    uint8_t green = (uint8_t)(((color >> 8) & 0xFF) * brightness);
                    uint8_t blue = (uint8_t)((color & 0xFF) * brightness);

                    LedStrip.setPixelColor(i, LedStrip.Color(red, green, blue));
                    LedStrip.show();
                    // Increment the frame to animate the gradient
                    frame = (frame + 5) % 360;  // Speed of gradient transition
                }
                } else {
                    LedStrip.fill((0,0,0));
                    LedStrip.show();
                }

        // Reduce the delay for faster updates
        vTaskDelay(taskRGBLed.getIntervalms() / portTICK_PERIOD_MS);  // Adjust delay to control the speed further
    }
}

// #include "RGBLed.h"
// #include <math.h>  // For the sin() function

// const int pinLed = 15;           // GPIO pin connected to the LED strip
// const int pixels = 92;           // Number of LEDs in the WS2815 strip

// Adafruit_NeoPixel LedStrip(pixels, pinLed, NEO_GRB + NEO_KHZ800);

// // Task function for alternating wave patterns
// void RGBLed(void *pvParameters) {
//     LedStrip.begin();
//     LedStrip.show();  // Initialize all pixels to 'off'

//     int frame = 0;  // Animation frame counter

//     while (true) {
//         // Update the colors on the strip for the current frame
//         for (int i = 0; i < LedStrip.numPixels(); i++) {
//             // Create a sinusoidal wave effect based on position and frame
//             float wave = (sin((i + frame) * 0.2) + 1.0) / 2.0;  // Range: 0 to 1
//             uint8_t red = 255 * wave * 0.9;  // Brightness reduced by 10%
//             uint8_t blue = 255 * (1.0 - wave) * 0.9;  // Opposite phase

//             // Set the pixel color alternating between red and blue
//             LedStrip.setPixelColor(i, LedStrip.Color(red, 0, blue));
//         }
//         LedStrip.show();

//         // Increment the frame for the next wave
//         frame++;

//         // Control the speed of the animation
//         vTaskDelay(taskRGBLed.getIntervalms() / portTICK_PERIOD_MS);  // Adjust delay to control the speed
//     }
// }

// #include "RGBLed.h"

// const int pinLed = 40;           // GPIO pin connected to the LED strip 40 and 41 are ok
// const int pixels = 32;           // Number of LEDs in the WS2815 strip

// Adafruit_NeoPixel LedStrip(pixels, pinLed, NEO_GRB + NEO_KHZ800);

// // Adjustable brightness (0.0 to 1.0, where 1.0 is full brightness)
// float brightness = 0.5;  // Default: 90% brightness

// // Task function for gradient pulsing pattern
// void RGBLed(void *pvParameters) {
//     LedStrip.begin();
//     LedStrip.show();  // Initialize all pixels to 'off'

//     int frame = 0;  // Animation frame counter

//     while (true) {
//         // Update the colors on the strip for the current frame
//         for (int i = 0; i < LedStrip.numPixels(); i++) {
//             // Create a gradient effect across the strip
//             int hue = (frame + (i * 360 / LedStrip.numPixels())) % 360;  // Color gradient
//             uint32_t color = LedStrip.ColorHSV(hue * 182);  // Convert hue to color

//             // Apply brightness adjustment
//             uint8_t red = (uint8_t)(((color >> 16) & 0xFF) * brightness);
//             uint8_t green = (uint8_t)(((color >> 8) & 0xFF) * brightness);
//             uint8_t blue = (uint8_t)((color & 0xFF) * brightness);

//             LedStrip.setPixelColor(i, LedStrip.Color(red, green, blue));
//         }
//         LedStrip.show();

//         // Increment the frame to animate the gradient
//         frame = (frame + 5) % 360;  // Speed of gradient transition

//         // Control the speed of the pulsing effect
//         vTaskDelay(taskRGBLed.getIntervalms() / portTICK_PERIOD_MS);
//     }
// }
