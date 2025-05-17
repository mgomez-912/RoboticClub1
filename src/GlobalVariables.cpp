#include "GlobalVariables.h"

TaskManager taskManager;
// TaskManager taskRGBLed;
TaskManager taskPrint;
TaskManager taskRXRead;
TaskManager taskMotorDriving;
TaskManager taskServot1;
TaskManager taskPWMExt;
// TaskManager taskLineFollow;
// TaskManager taskLineSense;
// TaskManager taskSBUSRead;

const int NUM_CHANNELS = 8;                                 // Number of channels to read
volatile unsigned int channelValues[NUM_CHANNELS]={0};

// Mutex for thread-safe access
portMUX_TYPE pidMux = portMUX_INITIALIZER_UNLOCKED;

//Status Line follower to handle cases
int statusLine = 0;
int position = 3500;
int lost_count = 0;
int lostCycles = 55;
int inter_count = 0;