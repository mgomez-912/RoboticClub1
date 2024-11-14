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
    taskRGBLed.taskCreation(RGBLed, "Core0_blinkRGB", 2048, 500, 2, 0);      // External Task
    taskRXRead.taskCreation(RXRead,"Core0_RXReader",2048,10,5,0);    
    
    // Configure the task to run on Core 1
    taskManager.taskCreation(myTask, "Core1_Task", 1024, 500, 1, 1); // Task created on main.cpp
    taskPrint.taskCreation(Printer, "Core1_Printer", 2048, 100,1,1); 
    taskMotorDriving.taskCreation(MotorDriving,"Core1_MotorDriving",2048,1000,5,1);
}

void loop()
{
    // Nothing in loop, tasks are handled by FreeRTOS
}