#ifndef __GLOBAL_VARIABLES_H__
#define __GLOBAL_VARIABLES_H__

#include <Arduino.h>
#include <TaskManager.h>

extern TaskManager taskManager;
// extern TaskManager taskRGBLed;
extern TaskManager taskPrint;
extern TaskManager taskRXRead;
extern TaskManager taskMotorDriving;
extern TaskManager taskServot1;
extern TaskManager taskPWMExt;
extern TaskManager taskLineFollow;
extern TaskManager taskLineSense;
// extern TaskManager taskSBUSRead;
extern TaskManager taskUltrasound;

extern const int NUM_CHANNELS;                // Number of channels to read
extern volatile unsigned int channelValues[]; // Array to store channel values

// Mutex for thread-safe access
extern portMUX_TYPE pidMux;

// Status Line follower to handle cases
extern int statusLine;
extern int position;
extern int lost_count; // Counter for the cycles when can see any line
extern int lostCycles;
extern int inter_count; // Counter for the intersection crossed
extern bool sensors[8];
extern bool inIntersection;

// Distance sensor
extern float distance; // Distance in cm

#endif