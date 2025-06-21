#ifndef TOF_DISTANCE_H
#define TOF_DISTANCE_H

#include <Arduino.h>
#include <HardwareSerial.h>

// --- Public Function Declarations ---

/**
 * @brief Initializes the TOF Sensor hardware and sends configuration commands.
 * This must be called once during setup.
 */
void initTOFDistanceSensor();

/**
 * @brief The FreeRTOS task function that continuously reads and parses data
 * from the TOF sensor.
 * @param pvParameters Standard FreeRTOS task parameters (not used).
 */
void TOFDistanceTask(void *pvParameters);

/**
 * @brief Safely gets the last valid distance measurement.
 * @return The distance in millimeters, or -1 if the reading is invalid.
 */
long getTOFDistance();


// --- Extern Variable Declarations ---
// These are defined in the .cpp file but declared here for external access.

extern volatile long tofDistanceMM;
extern HardwareSerial TOFSerial;


#endif // TOF_DISTANCE_H