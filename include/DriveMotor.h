#ifndef DRIVE_MOTOR_H
#define DRIVE_MOTOR_H

#include <Arduino.h>
#include "GlobalVariables.h"

void MotorDriving(void *pvParameters);

void decision(int signal1, int signal2);
void forward(int speed, int signal2);
void reverse(int speed, int signal2);
void turn(bool dir1, bool dir2, int speed1, int speed2);
void M1(bool direction,int speed, bool motorNumber);
void brake(int motor);
void stop(int motor, int del);

#endif