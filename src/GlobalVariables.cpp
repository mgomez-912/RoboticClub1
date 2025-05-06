#include "GlobalVariables.h"

TaskManager taskManager;
TaskManager taskRGBLed;
TaskManager taskPrint;
TaskManager taskRXRead;
TaskManager taskMotorDriving;
TaskManager taskServot1;
TaskManager taskPWMExt;
TaskManager taskLineFollow;

const int NUM_CHANNELS = 6;                                 // Number of channels to read
volatile unsigned int channelValues[NUM_CHANNELS]={0};

// Mutex for thread-safe access
portMUX_TYPE pidMux = portMUX_INITIALIZER_UNLOCKED;

//Status Line follower to handle cases
int statusLine = 0;