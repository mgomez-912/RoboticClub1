#include "PWMExtender.h"

// Initialize the PWM driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int currentSpeed = 0;
int targetSpeed = 0;

const int openClaw = 160;
const int closedClaw = 105;
const int front = 56; 
const int back = 180;

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
  delay(1000);
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

void armPos(int rotation, int angle, int shoulder, int elbow, int wrist, int claw, int del){
    setServoAngle(2, shoulder);
    setServoAngle(3, elbow);
    setServoAngle(1, angle);
    vTaskDelay(del);
    setServoAngle(4, wrist);
    setServoAngle(5, claw);
    setServoAngle(0, rotation);
    vTaskDelay(del);
    
}

void inputHandle(){
    if(channelValues [7]>1500){
        if (channelValues[4]>1500) armPos(front, 175, 10, 165, 135, openClaw, 150); //armPos(back,150,0,0,0,0,10);
        // else armPos(back,20,0,0,0,0,10);
        else armPos(front, 95, 65, 95, 135, openClaw, 150); //armPos (back, 110, 110, 70, 10, openClaw,10);
    }else{
        if (channelValues[4]>1500) armPos(front, 175, 10, 165, 135, closedClaw, 150); //armPos(back,150,0,0,0,0,10);
        // else armPos(back,20,0,0,0,0,10);
        else armPos (front, 95, 65, 95, 135, closedClaw, 150);
    }

}

//Pick1 armPos(back, 20,115,90,10,closedClaw,10) backward
//pick1 armPos(front, 175, 20, 160, 135, openClaw, 150) forward
//delivery armPos(back, 110, )

/*
to pickup: first decrease the angle in 2 steps, then go for the pickup position
to storage: from pick up, first increase angle to a high enough position, then make the following movements depending on where you want to go
to deliver: go directly from both cases.

*/