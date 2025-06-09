#include "PWMExtender.h"

// Initialize the PWM driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
/////////////////////////////////////////
int prevCh4 = -1;
int prevCh5 = -1;
int prevCh6 = -1;

bool currentTargetArm = false;
int currentArmElbow = 100;
int fix = 0;
/////////////////////////////////////////
// Storage positions: rotation, angle, shoulder, elbow, wrist
int storagePos[3][5] = {
    {front-5, 105, 155, 130, 135},  // Storage 1
    {front, 90, 65, 85, 135},  // Storage 2
    {170, 80, 80, 85, 125}   // Storage 3
};
int deliveryPos[5] = {front, 90, 70, 75, 135};
int liftUpPos[5] = {front, 120, 20, 145, 135};  // safe transition pose

int pickPos1[5] = {front, 180, 10, 170, 135}; 
int pickPos2[5]  = {front, 160, 30, 100, 135};
//pick1 armPos(front, 175, 20, 165, 135, pauseMov) forward

void PWMExtender(void *pvParameters) {
    initializeServo(0);
    initializeServo(1);
    initializeServo(2);
    initializeServo(3);
    initializeServo(4);
    initializeServo(5);
    vTaskDelay(10);

    while(true) {
        inputHandle();
        
        vTaskDelay(taskPWMExt.getIntervalms() / portTICK_PERIOD_MS);  // Delay for the blink time
    }

}

// Function to initialize the PCA9685
// In TaskServoPCA.cpp - modify initializeServo()
void initializeServo(int servoChannel) {
  Serial.begin(115200);
  while(!Serial); // Wait for serial monitor in debug mode

  // Serial.printf("Using I2C pins: SDA=%d, SCL=%d\n", SDA_PIN, SCL_PIN);

  Wire.begin(SDA_PIN, SCL_PIN);
  if(!pwm.begin()) {
    Serial.println("PCA9685 not found!");
    while(1); // Halt if initialization fails
  }
  
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
//   Serial.printf("PCA9685 initialized at %dHz\n", SERVO_FREQ);
  
  // Test communication
  pwm.setPWM(servoChannel, 0, 0); // Start with servo off
  delay(150);
}

// Function to set servo angle
void setServoAngle(uint8_t channel, uint8_t angle) {
  // Constrain angle to 0-180 degrees
  angle = constrain(angle, 0, 180);
  
  // Map angle to pulse width
  uint16_t pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  
  // Set PWM
  pwm.setPWM(channel, 0, pulse);
  
  // Optional: Print debug info
  // Serial.printf("Channel %d set to %d degrees (pulse: %d)\n", channel, angle, pulse);
}

/////////////////////////////////
ArmTarget getArmTargetFromSwitches() {
    int ch4 = channelValues[4];
    int ch5 = channelValues[5];
    int ch6 = channelValues[6];

    if (ch4 > deadBHigh && ch5 < deadBLow) {
        if (ch6 < deadBLow) return PICKUP_2;
        else if (ch6 > deadBHigh) return DELIVERY;
    } else 
    if (ch4 > deadBHigh && ch5 > deadBHigh) {            //originally after the first else
        if (ch6 < deadBLow) return STORAGE;
        if (ch6 > deadBHigh) return DELIVERY;
    }else {
        if (ch6 < deadBLow) return PICKUP_1;
        else if (ch6 > deadBHigh) return DELIVERY;
    }
    return NONE;
}



void armPos(int rotation, int angle, int shoulder, int elbow, int wrist, int del){
    currentArmElbow = elbow;
    setServoAngle(1, angle);
    vTaskDelay(del);
    setServoAngle(2, shoulder);
    setServoAngle(3, elbow);
    vTaskDelay(del);
    setServoAngle(4, wrist);
    // setServoAngle(5, claw);
    setServoAngle(0, rotation);
    vTaskDelay(del);   
}

void inputHandle(){
    if(channelValues[7]>1500) setServoAngle(5,openClaw);
    else setServoAngle(5,closedClaw);

    // updateArmAngleWithFix();  // Dynamically adjust the angle every loop
    processArmPosition();
}

//Pick1 armPos(back, 20,115,90,10,pauseMov) backward
//pick1 armPos(front, 175, 20, 165, 135, pauseMov) forward
//armPos(front, 95, 65, 95, 135, pauseMov);

/*
to pickup: first decrease the angle in 2 steps, then go for the pickup position
to storage: from pick up, first increase angle to a high enough position, then make the following movements depending on where you want to go
to deliver: go directly from both cases.  
*/

void moveArmSafely(int to[5]) {
    armPos(liftUpPos[0], liftUpPos[1], liftUpPos[2], liftUpPos[3], liftUpPos[4], pauseMov);
    vTaskDelay(150);
    armPos(to[0], to[1], to[2], to[3], to[4], pauseMov);
}

ArmTarget previousTarget = NONE;

void processArmPosition() {
    ArmTarget currentTarget = getArmTargetFromSwitches();

    // Only move if the target has changed
    if (currentTarget == previousTarget || currentTarget == NONE) return;

    // Define transitions needing lift-up
    bool needsTransition = false;

    if ((previousTarget == DELIVERY || currentTarget == STORAGE) && (currentTarget == PICKUP_1 || currentTarget == PICKUP_2))
        needsTransition = true;
    // else if ((previousTarget == PICKUP_1 || previousTarget == PICKUP_2) && (currentTarget == DELIVERY || currentTarget == STORAGE))
    //         needsTransition = true;

    // Go to lift-up first if needed
    if (needsTransition) {
        armPos(liftUpPos[0], liftUpPos[1], liftUpPos[2], liftUpPos[3], liftUpPos[4], pauseMov);
        vTaskDelay(150);
    }

    // Move to final target
    switch (currentTarget) {
        case PICKUP_1:
            setServoAngle(3, pickPos1[3]+10);          //To avoid hitting the floor
            setServoAngle(2,pickPos1[2]+10);
            vTaskDelay(pauseMov*4);
            armPos(pickPos1[0], pickPos1[1], pickPos1[2], pickPos1[3], pickPos1[4], pauseMov);
            currentTargetArm = false;
            break;

        case PICKUP_2:
            armPos(pickPos2[0], pickPos2[1], pickPos2[2], pickPos2[3], pickPos2[4], pauseMov);
            break;

        case DELIVERY:
            armPos(deliveryPos[0], deliveryPos[1], deliveryPos[2], deliveryPos[3], deliveryPos[4], pauseMov);
            currentTargetArm = true;
            break;
        
        case STORAGE:
            // armPos(storagePos[0][0],storagePos[0][1],storagePos[0][2],storagePos[0][3],storagePos[0][4], pauseMov);
            armPos(storagePos[1][0],storagePos[1][1],storagePos[1][2],storagePos[1][3],storagePos[1][4], pauseMov);
            break;

        default:
            break;
    }
    // Update previous state
    previousTarget = currentTarget;
}

// void updateArmAngleWithFix() {
//     int adjusted;
//     // Handle position change
//     fix=map(channelValues[5], 990, 2010, 0, 10);
//     if(currentTargetArm == true)  adjusted = constrain(currentArmElbow + fix, 0, 200);
//     if(currentTargetArm == false) adjusted = constrain(currentArmElbow - fix, 0, 200);
//     setServoAngle(3, adjusted);
// }