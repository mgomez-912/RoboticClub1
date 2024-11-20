#include "DriveMotor.h"

// Motor control pins
const int M1_Forward = 4;
const int M1_Reverse = 5;
const int M2_Forward = 6;
const int M2_Reverse = 7;

const int motorProt = 500; // Default protection time for motor direction change
const int speedlim = 100;

// Channel values
const int minres = 1425;
const int maxres = 1575;

// Ramp-up configuration
const int rampStep = 5; // Step size for speed increase
const int rampDelay = 5; // Delay in milliseconds for each ramp step

bool isFirstRun = true; // Tracks if this is the first run

// State definitions
enum RobotState { IDLE, FORWARD, REVERSE, TURN_LEFT, TURN_RIGHT };
RobotState currentState = IDLE;
RobotState lastState = IDLE; // Track the previous state for transitions

// FreeRTOS tick-based timer
TickType_t idleStartTick = 0;
bool idleTimerRunning = false;

// Helper variables
bool lastDirM1 = true; // True: Forward, False: Reverse
bool lastDirM2 = true;
int currentSpeedM1 = 0; // Current speed of Motor 1
int currentSpeedM2 = 0; // Current speed of Motor 2

// Function prototypes
void updateState(int advSignal, int turnSignal);
void executeState();
void stopMotors();
void executeActionImmediately();
void setMotorDirection(bool dirM1, int targetSpeedM1, bool dirM2, int targetSpeedM2);
void rampSpeed(int &currentSpeed, int targetSpeed, int controlPinForward, int controlPinReverse);
void controlForward();
void controlReverse();

void MotorDriving(void *pvParameters) {
    if (isFirstRun) {
        stopMotors(); // Ensure all motors are stopped
        isFirstRun = false; // Reset the first-run flag
        vTaskDelay(500 / portTICK_PERIOD_MS); // Optional delay to ensure stability
    }

    while (true) {
        // Read input signals (e.g., from receiver or other input source)
        int advSignal = channelValues[1]; // Forward/Reverse
        int turnSignal = channelValues[3]; // Left/Right

        // Update state based on input
        updateState(advSignal, turnSignal);

        // Execute actions for the current state
        executeState();

        // Small delay for stability
        vTaskDelay(taskMotorDriving.getIntervalms() / portTICK_PERIOD_MS);
    }
}


void updateState(int advSignal, int turnSignal) {
    // Determine current state based on input signals
    if (advSignal > minres && advSignal < maxres && turnSignal > minres && turnSignal < maxres) {
        currentState = IDLE;

        if (!idleTimerRunning) {
            idleStartTick = xTaskGetTickCount(); // Record the tick count when IDLE starts
            idleTimerRunning = true;
        }
    } else {
        idleTimerRunning = false;
    }

    if (advSignal >= maxres) {
        currentState = FORWARD;
    } else if (advSignal <= minres) {
        currentState = REVERSE;
    } else if (turnSignal >= maxres) {
        currentState = TURN_RIGHT;
    } else if (turnSignal <= minres) {
        currentState = TURN_LEFT;
    }
}

void executeState() {
    switch (currentState) {
        case IDLE:
            stopMotors();
            break;

        case FORWARD:
            controlForward();
            break;

        case REVERSE:
            controlReverse();
            break;

        case TURN_RIGHT:
            setMotorDirection(true, map(channelValues[3], maxres, 2005, 0, speedlim),
                              false, map(channelValues[3], maxres, 2005, 0, speedlim));
            break;

        case TURN_LEFT:
            setMotorDirection(false, map(channelValues[3], minres, 995, 0, speedlim),
                              true, map(channelValues[3], minres, 995, 0, speedlim));
            break;
    }

    // Track the previous state for debugging or future use
    lastState = currentState;
}


void executeActionImmediately() {
    switch (currentState) {
        case FORWARD:
            controlForward();
            break;
        case REVERSE:
            controlReverse();
            break;
        case TURN_RIGHT:
            setMotorDirection(true, map(channelValues[3], maxres, 2005, 0, speedlim),
                              false, map(channelValues[3], maxres, 2005, 0, speedlim));
            break;
        case TURN_LEFT:
            setMotorDirection(false, map(channelValues[3], minres, 995, 0, speedlim),
                              true, map(channelValues[3], minres, 995, 0, speedlim));
            break;
    }
}

void controlForward() {
    int baseSpeed = map(channelValues[1], maxres, 2005, 0, speedlim);
    int adjustment = 0;

    if (channelValues[3] > maxres) {
        adjustment = map(channelValues[3], maxres, 2005, 0, baseSpeed);
        setMotorDirection(true, baseSpeed, true, baseSpeed - adjustment);
    } else if (channelValues[3] < minres) {
        adjustment = map(channelValues[3], minres, 995, 0, baseSpeed);
        setMotorDirection(true, baseSpeed - adjustment, true, baseSpeed);
    } else {
        setMotorDirection(true, baseSpeed, true, baseSpeed);
    }
}

void controlReverse() {
    int baseSpeed = map(channelValues[1], minres, 995, 0, speedlim);
    int adjustment = 0;

    if (channelValues[3] > maxres) {
        adjustment = map(channelValues[3], maxres, 2005, 0, baseSpeed);
        setMotorDirection(false, baseSpeed, false, baseSpeed - adjustment);
    } else if (channelValues[3] < minres) {
        adjustment = map(channelValues[3], minres, 995, 0, baseSpeed);
        setMotorDirection(false, baseSpeed - adjustment, false, baseSpeed);
    } else {
        setMotorDirection(false, baseSpeed, false, baseSpeed);
    }
}

void stopMotors() {
    analogWrite(M1_Forward, LOW);
    analogWrite(M1_Reverse, LOW);
    analogWrite(M2_Forward, LOW);
    analogWrite(M2_Reverse, LOW);
    currentSpeedM1 = 0;
    currentSpeedM2 = 0;
}

void setMotorDirection(bool dirM1, int targetSpeedM1, bool dirM2, int targetSpeedM2) {
    // Stop both motors if either motor direction is changing
    if (dirM1 != lastDirM1 || dirM2 != lastDirM2) {
        stopMotors(); // Stop both motors
        vTaskDelay(motorProt / portTICK_PERIOD_MS); // Brief delay to ensure motors are fully stopped
    }

    // Update Motor 1
    rampSpeed(currentSpeedM1, targetSpeedM1, dirM1 ? M1_Forward : M1_Reverse, dirM1 ? M1_Reverse : M1_Forward);
    lastDirM1 = dirM1;

    // Update Motor 2
    rampSpeed(currentSpeedM2, targetSpeedM2, dirM2 ? M2_Forward : M2_Reverse, dirM2 ? M2_Reverse : M2_Forward);
    lastDirM2 = dirM2;
}

void rampSpeed(int &currentSpeed, int targetSpeed, int controlPinForward, int controlPinReverse) {
    while (currentSpeed != targetSpeed) {
        if (currentSpeed < targetSpeed) {
            currentSpeed += rampStep;
            if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
        } else {
            currentSpeed -= rampStep;
            if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
        }
        analogWrite(controlPinForward, currentSpeed);
        analogWrite(controlPinReverse, LOW);
        vTaskDelay(rampDelay / portTICK_PERIOD_MS);
    }
}


//****************************************** */
// #include "PWMExtender.h"

// const int lim = 200;         //speed limit
// const int motorDel = 50;

// //all pins must be PWM enabled pin with ~ printed beside them
// const int M1_Forward = 4;
// const int M1_Reverse = 5;
// const int M2_Forward = 6;
// const int M2_Reverse = 7;

// int c=0;                //decision cases
// int baseSpeed = 0;

// //direction states
// bool CW = true;
// bool CCW = false;
// bool lastDirM1;
// bool lastDirM2;

// //deadpoints
// int minres=1400;
// int maxres=1600;

// //channels
// int adv1=1500;           //RC control Channel 2
// int turn1=1500;          //RC control Channel 4

// void MotorDriving(void *pvParameters){


//     while(true){
//         //ChannelValues[Channel-1]**********************************************************************
//         adv1 = channelValues[3];        // Read Channel 2 (forward, reverse)
//         turn1 = channelValues[1];       // Read Channel 4 (turn right, left)

//         // if(adv1>1520){
//         //   analogWrite(M1_Forward,map(adv1,1520,2005,0,255));
//         // } else{analogWrite(M1_Forward,0);}
//         // if(adv1<1480){
//         //   analogWrite(M1_Reverse,map(adv1,1480,995,0,255));
//         // } else{analogWrite(M1_Reverse,0);}
//         // if(turn1>1520){
//         //   analogWrite(M2_Forward,map(turn1,1520,2005,0,255));
//         // } else{analogWrite(M2_Forward,0);}
//         // if(turn1<1480){
//         //   analogWrite(M2_Reverse,map(turn1,1480,995,0,255));
//         // } else{analogWrite(M2_Reverse,0);}

//         // analogWrite(M1_Forward,map(channelValues[0],1010,2000,0,255));
//         // analogWrite(M1_Reverse,map(channelValues[1],1010,2000,0,255));
//         // analogWrite(M2_Forward,map(channelValues[2],1010,2000,0,255));
//         // analogWrite(M2_Reverse,map(channelValues[3],1010,2000,0,255));

//         decision(adv1, turn1);                      //Motors function

//         vTaskDelay(taskMotorDriving.getIntervalms() / portTICK_PERIOD_MS);
//     }
// }

// void decision(int signal1, int signal2){
//   if(signal1>minres && signal1<maxres){
//     c=1;
//     if(signal2<minres || signal2>maxres){
//       c=4;
//     }
//   }
//   else if(signal1>maxres){
//     c=2;
//   }
//   else{
//     c=3;
//   }

//   switch (c) {
//   case 1:
//       for(int i=4; i<9; i++){
//         analogWrite(i,0);
//       }
//     break;
//   case 2:
//       analogWrite(M1_Forward,map(adv1,maxres,2005,0,255));
//     break;
//   case 3:
//       analogWrite(M1_Reverse,map(adv1,minres,995,0,255));
//     break;
//   case 4:
//       if(signal2<minres){
//         analogWrite(M2_Reverse,map(turn1,minres,995,0,255));
//       }
//       else if(signal2>maxres){
//         analogWrite(M2_Forward,map(turn1,maxres,2005,0,255));
//       }
//     break;
//   default:
//     stop(3,50);
//     break;
//   }
// }
//***************************************************************** */

// #include "DriveMotor.h"
// #include "PWMExtender.h"

// const int lim = 255;         //speed limit
// const int motorDel = 50;

// //all pins must be PWM enabled pin with ~ printed beside them
// const int M1_Forward = 0;
// const int M1_Reverse = 1;
// const int M2_Forward = 2;
// const int M2_Reverse = 3;

// int c=0;                //decision cases
// int baseSpeed = 0;

// //direction states
// bool CW = true;
// bool CCW = false;
// bool lastDirM1;
// bool lastDirM2;

// //deadpoints
// int minres=1400;
// int maxres=1600;

// //channels
// int adv1=1500;           //RC control Channel 2
// int turn1=1500;          //RC control Channel 4

// void MotorDriving(void *pvParameters){
//     // pinMode(M1_Forward, OUTPUT);
//     // pinMode(M1_Reverse, OUTPUT);
//     // pinMode(M2_Forward, OUTPUT);
//     // pinMode(M2_Reverse, OUTPUT);

//     while(true){
//         //ChannelValues[Channel-1]**********************************************************************
//         adv1 = channelValues[1];        // Read Channel 2 (forward, reverse)
//         turn1 = channelValues[3];       // Read Channel 4 (turn right, left)

//         decision(adv1, turn1);                      //Motors function

//         vTaskDelay(taskMotorDriving.getIntervalms() / portTICK_PERIOD_MS);
//     }
// }

// void decision(int signal1, int signal2){
//   if(signal1>minres && signal1<maxres){
//     c=1;
//     if(signal2<minres || signal2>maxres){
//       c=4;
//     }
//   }
//   else if(signal1>maxres){
//     c=2;
//   }
//   else{
//     c=3;
//   }

//   switch (c) {
//   case 1:
//       baseSpeed = 0;
//       brake(motorDel);
//     break;
//   case 2:
//       baseSpeed = map(signal1,1500,2020,0,lim);
//       forward(baseSpeed,signal2);
//     break;
//   case 3:
//     baseSpeed = map(signal1,1450,980,0,lim); 
//       reverse(baseSpeed,signal2);
//     break;
//   case 4:
//       if(signal2<minres){
//         baseSpeed=map(signal2,1450,980,0,lim);
//         turn(CW, CCW, baseSpeed, baseSpeed);
//       }
//       else if(signal2>maxres){
//         baseSpeed=map(signal2,1550,2020,0,lim);
//         turn(CCW, CW, baseSpeed, baseSpeed);
//       }
//     break;
//   default:
//     stop(3,50);
//     break;
//   }
// }

// void forward(int speed, int signal2){
//   if(signal2>minres && signal2<maxres){
//     turn(CW, CW, speed, speed);
//     }
//     else if(signal2<minres){
//       int turnL = (speed)-(map(signal2,1450,980,0,lim)*0.75);
//       if(turnL>0 && turnL<lim){
//         turn(CW, CW, speed, turnL);
//       }
//     }
//     else if(signal2>maxres){
//       int turnR = (speed)-(map(signal2,1550,2020,0,lim)*0.75);
//       if(turnR>0 && turnR<lim){
//         turn(CW, CW, turnR, speed);
//       }
//     }
// }


// void reverse(int speed, int signal2){
//   if(signal2>minres && signal2<maxres){
//     turn(CCW, CCW, speed, speed);
//   }
//   else if(signal2<minres){
//     int turnL = (speed)-(map(signal2,1450,980,0,lim)/2);
//     if(turnL>0 && turnL<lim){
//       turn(CCW, CCW, speed, turnL);
//     }
//   }
//   else if(signal2>maxres){
//     int turnR = (speed)-(map(signal2,1550,2020,0,lim)/2);
//     if(turnR>0 && turnR<lim){
//       turn(CCW, CCW, turnR, speed);
//     }
//   }
// }

// void turn(bool dir1, bool dir2, int speed1, int speed2){
//   M1(dir1, speed1, 1);
//   M1(dir2, speed2, 2);
// }

// void M1(bool direction,int speed, bool motorNumber)
// {
//     if(motorNumber == 1){
//         if(speed<0 || speed>255){
//             vTaskDelay(1);
//         }
//         else if(direction != lastDirM1){
//             stop(3,500);
//         }
//         else if(direction == CCW)
//         {
//         PWMExtender(M1_Forward,speed);
//         PWMExtender(M1_Reverse,0);
//         lastDirM1=CCW;    
//         }else{
//         PWMExtender(M1_Reverse,speed);
//         PWMExtender(M1_Forward,0);  
//         lastDirM1=CW;    
//         }
//     }
//     if(motorNumber == 2){
//         if(speed<0 || speed>255){
//             vTaskDelay(1);
//         }
//         else if(direction != lastDirM2){
//             stop(3,500);
//         }
//         else if(direction == CW)
//         {
//         PWMExtender(M2_Forward,speed);
//         PWMExtender(M2_Reverse,0);
//         lastDirM2=CW;   
//         }else{
//         PWMExtender(M2_Reverse,speed);
//         PWMExtender(M2_Forward,0); 
//         lastDirM2=CCW;    
//         }
//     }
// }

// void brake(int motor)
// {
//    if(motor == 1)
//   {
//     PWMExtender(M1_Forward,255);
//     PWMExtender(M1_Reverse,255);   
//   }else if(motor == 2){
//     PWMExtender(M2_Forward,255);
//     PWMExtender(M2_Reverse,255);     
//   } else{
//     PWMExtender(M1_Forward,255);
//     PWMExtender(M1_Reverse,255);
//     PWMExtender(M2_Forward,255);
//     PWMExtender(M2_Reverse,255);  
//   }
// }

// void stop(int motor, int del)
// {
//    if(motor == 1)
//   {
//    PWMExtender(M1_Forward,0);
//    PWMExtender(M1_Reverse,0);  
//   }else if(motor == 2){
//    PWMExtender(M2_Forward,0);
//    PWMExtender(M2_Reverse,0);  
//   } else{
//     PWMExtender(M1_Forward,0);
//     PWMExtender(M1_Reverse,0);
//     PWMExtender(M2_Forward,0);
//     PWMExtender(M2_Reverse,0);
//   }
//   delay(del);
// }