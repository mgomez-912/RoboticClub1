#include "GlobalVariables.h"

TaskManager taskManager;
TaskManager taskRGBLed;
TaskManager taskPrint;
TaskManager taskRXRead;
TaskManager taskMotorDriving;
TaskManager taskServot1;
TaskManager taskPWMExt;
TaskManager taskLineFollow;
TaskManager taskColorSensor;
TaskManager taskConveyorBeltMotor;

const int NUM_CHANNELS = 6;                                 // Number of channels to read
volatile unsigned int channelValues[NUM_CHANNELS]={0};