#include "main.h"

// Task function
void myTask(void *pvParameters)
{
    // Your task code here
    Serial.println("Running Task...");
}


void setup()
{
    Serial.begin(115200);

    // Configure the task to run on Core 0
    // task.taskCreation(Function, Name, Size, sleepTime, priority (0-5), core);
    // taskRGBLed.taskCreation(RGBLed, "Core0_blinkRGB", 2048, 500, 2, 0);      // External Task
    taskServot1.taskCreation(Servot1, "Core0_sweepServo", 2048, 100, 2, 0);
    // taskTesting.taskCreation(taskTest,"Core0_blink",2048,1000,1,0);    
    
    // Configure the task to run on Core 1
    // taskManager.taskCreation(myTask, "Core1_Task", 1024, 500, 1, 1); // Task created on main.cpp
    // taskPrint.taskCreation(Printer, "Core1_Printer", 2048, 100,1,1); 
}

void loop()
{
    // Nothing in loop, tasks are handled by FreeRTOS
}

// #define PPM_PIN 5           // Pin connected to the PPM signal
// #define NUM_CHANNELS 6      // Number of channels to read
// #define SYNC_GAP 3000       // Minimum time (in microseconds) to consider as a sync gap
// #define MIN_PULSE_WIDTH 900 // Minimum pulse width (in microseconds) for a valid signal
// #define MAX_PULSE_WIDTH 2100 // Maximum pulse width (in microseconds) for a valid signal

// volatile unsigned long lastTime = 0;
// volatile unsigned int channelValues[NUM_CHANNELS];  // Array to store channel values
// volatile int currentChannel = 0;

// void IRAM_ATTR ppmInterruptHandler() {
//     unsigned long currentTime = micros();
//     unsigned long duration = currentTime - lastTime;
//     lastTime = currentTime;

//     if (duration > SYNC_GAP) {
//         // Detected sync pulse, reset to the first channel
//         currentChannel = 0;
//     } else if (duration >= MIN_PULSE_WIDTH && duration <= MAX_PULSE_WIDTH && currentChannel < NUM_CHANNELS) {
//         // If duration is within valid range, store it as a channel value
//         channelValues[currentChannel] = duration;
//         currentChannel++;
//     }
// }

// void setup() {
//     Serial.begin(115200);
//     pinMode(PPM_PIN, INPUT_PULLUP);
//     attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterruptHandler, FALLING);

//     // Initialize channel values to a default state
//     for (int i = 0; i < NUM_CHANNELS; i++) {
//         channelValues[i] = 0;
//     }
// }

// void loop() {
//     // Print the most recent values for each channel
//     for (int i = 0; i < NUM_CHANNELS; i++) {

//         Serial.print(channelValues[i]);
//         Serial.print(" \t");
//     }
//     Serial.println();

//     delay(20);  // Short delay to limit serial output rate
// }
