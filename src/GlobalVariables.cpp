#include "GlobalVariables.h"

TaskManager taskManager;
TaskManager taskRGBLed;
TaskManager taskPrint;
TaskManager taskRXRead;
TaskManager taskMotorDriving;

const int NUM_CHANNELS = 6;                                 // Number of channels to read
volatile unsigned int channelValues[NUM_CHANNELS];