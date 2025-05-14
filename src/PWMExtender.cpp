#include "PWMExtender.h"

// Initialize the PWM driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
/////////////////////////////////////////
int prevCh4 = -1;
int prevCh5 = -1;
int prevCh6 = -1;


/////////////////////////////////////////
// Storage positions: rotation, angle, shoulder, elbow, wrist
int storagePos[3][5] = {
    {130, 90, 70, 95, 135},  // Storage 1
    {150, 85, 75, 90, 130},  // Storage 2
    {170, 80, 80, 85, 125}   // Storage 3
};
int deliveryPos[5] = {front, 95, 60, 90, 135};
int liftUpPos[5] = {front, 120, 20, 145, 135};  // safe transition pose

int pickPos1[5] = {front, 167, 0, 160, 135}; 
int pickPos2[5]  = {front, 140, 10, 100, 135};
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
  Serial.printf("PCA9685 initialized at %dHz\n", SERVO_FREQ);
  
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
    int ch6 = channelValues[6];

    if (ch4 > deadBHigh) {
        if (ch6 < deadBLow) return PICKUP_2;
        else if (ch6 > deadBHigh) return DELIVERY;
    } else {
        if (ch6 < deadBLow) return PICKUP_1;
        else if (ch6 > deadBHigh) return DELIVERY;
    }

    return NONE;
}



void armPos(int rotation, int angle, int shoulder, int elbow, int wrist, int del){
    setServoAngle(1, angle);
    vTaskDelay(del);
    setServoAngle(2, shoulder);
    vTaskDelay(del);
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

    // Handle position change
    processArmPosition();
    
    // if (channelValues[4]>1500){
    //   armPos(front, 175, 10, 165, 135, pauseMov); //pick up
    // }
    // else{
    //   // setServoAngle(1,135);
    //   vTaskDelay(pauseMov);
    //   armPos(front, 95, 65, 105, 135, pauseMov); //armPos (back, 110, 110, 70, 10, openClaw,10);
    // } 
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
    vTaskDelay(200);
    armPos(to[0], to[1], to[2], to[3], to[4], pauseMov);
}

ArmTarget previousTarget = NONE;

void processArmPosition() {
    ArmTarget currentTarget = getArmTargetFromSwitches();

    // Only move if the target has changed
    if (currentTarget == previousTarget || currentTarget == NONE) return;

    // Define transitions needing lift-up
    bool needsTransition = false;

    if ((previousTarget == PICKUP_1 || previousTarget == PICKUP_2) && currentTarget == DELIVERY)
        needsTransition = true;
    else if (previousTarget == DELIVERY && (currentTarget == PICKUP_1 || currentTarget == PICKUP_2))
        needsTransition = true;

    // Go to lift-up first if needed
    if (needsTransition) {
        armPos(liftUpPos[0], liftUpPos[1], liftUpPos[2], liftUpPos[3], liftUpPos[4], pauseMov);
        vTaskDelay(200);
    }

    // Move to final target
    switch (currentTarget) {
        case PICKUP_1:
            setServoAngle(3, pickPos1[3]+15);          //To avoid hitting the floor
            setServoAngle(2,pickPos1[2]+15);
            vTaskDelay(pauseMov*4);
            armPos(pickPos1[0], pickPos1[1], pickPos1[2], pickPos1[3], pickPos1[4], pauseMov);
            break;

        case PICKUP_2:
            armPos(pickPos2[0], pickPos2[1], pickPos2[2], pickPos2[3], pickPos2[4], pauseMov);
            break;

        case DELIVERY:
            armPos(deliveryPos[0], deliveryPos[1], deliveryPos[2], deliveryPos[3], deliveryPos[4], pauseMov);
            break;

        default:
            break;
    }

    // Update previous state
    previousTarget = currentTarget;
}



// void processArmPosition() {
//     int ch4 = channelValues[4];
//     int ch5 = channelValues[5];
//     int ch6 = channelValues[6];

//     // Check if switches have changed
//     if (ch4 == prevCh4 && ch5 == prevCh5 && ch6 == prevCh6) return;

//     // Update previous values
//     prevCh4 = ch4;
//     prevCh5 = ch5;
//     prevCh6 = ch6;

//     // Logic for Pick and Deliver
//     if (ch4 > deadBHigh) {
//         if (ch6 < deadBLow) {
//             // Pick Position 1
//             moveArmSafely(pickPos1);
//         } else if (ch6 > deadBHigh) {
//             moveArmSafely(deliveryPos);
//         }
//     }
//     else if (ch4 < deadBLow && ch5 >= deadBHigh) {
//         if (ch6 < deadBLow) {
//             // Pick Position 1
//             moveArmSafely(pickPos1);
//         } else if (ch6 > deadBHigh) {
//             moveArmSafely(deliveryPos);
//         }
//     }
//     else if (ch4 < deadBLow && ch5 < deadBLow) {
//         if (ch6 < deadBLow) {
//             moveArmSafely(storagePos[0]);
//         }
//         else if (ch6 >= deadBHigh && ch6 <= deadBLow) {
//             moveArmSafely(storagePos[1]);
//         }
//         else if (ch6 > deadBHigh) {
//             moveArmSafely(storagePos[2]);
//         }
//     }
// }
