#include "main.h"

void setup()
{
    Serial.begin(115200);

    // Configure the task to run on Core 0
    // task.taskCreation(Function, Name, Size, sleepTime, priority (0-5), core);
    // taskRGBLed.taskCreation(RGBLed, "Core0_blinkRGB", 2048, 50, 2, 0);      // External Task
    taskRXRead.taskCreation(RXRead,"Core0_RXReader",2048,20,5,0);
    // taskSBUSRead.taskCreation(SBUSRead, "Core0_SBUSRead",4096, 20, 5, 0); 
    taskPWMExt.taskCreation(PWMExtender,"Core0_PWMExtender",3000,50,3,0); 
    // taskLineSense.taskCreation(LineSense, "Core0_LineSense", 2048,5,5,0); 
    
    // Configure the task to run on Core 1
    // taskPrint.taskCreation(Printer, "Core1_Printer", 2048, 100,1,1); 
    // taskLineFollow.taskCreation(LineFollow,"Core0_Line",2048,15,5,1); 
    taskMotorDriving.taskCreation(MotorDriving,"Core1_MotorDriving",4096,20,5,1);
    // taskServot1.taskCreation(Servot1, "Core1_sweepServo", 2048, 500, 2, 1); 
}

void loop()
{
    // Nothing in loop, tasks are handled by FreeRTOS
}