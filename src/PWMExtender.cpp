#include "PWMExtender.h"
#include "ColorSensor.h"

// Initialize the PWM driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int currentSpeed = 0;
int targetSpeed = 0;

void PWMExtender(void *pvParameters) {
    initializeServo(0);
    //initializeServo(5);

    while(true) {
      // Sweep the servo back and forth
        // Serial.println("Sweeping servo...");
        sweepServoByColor(0); 
        //sweepServo(0);
        // sweepServo(5);
        
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

// Function to sweep the servo
//void sweepServo(int servoChannel) {
  //Serial.println("Moving servo from 0 to 180 degrees");
  
  // Move from 0 to 180 degrees
  //for (int angle = 0; angle <= 180; angle++) {
    //setServoAngle(servoChannel, angle);
    //delay(15);
  //}
  
  //delay(500);
  
  //Serial.println("Moving servo from 180 to 0 degrees");
  
  // Move from 180 to 0 degrees
  //for (int angle = 180; angle >= 0; angle--) {
    //setServoAngle(servoChannel, angle);
    //delay(15);
  //}
  
  //delay(500);
//}

// Function to sweep the servo based on color
uint8_t neutralpos = 45;
void sweepServoByColor(int servoChannel) {
  uint8_t targetAngle;

  if (detectedColor == "blue") {
      targetAngle = 45;
      Serial.println("BLUE selected: Moving to 90°");
  }
  else if (detectedColor == "red") {
      targetAngle = 0;
      Serial.println("RED selected: Moving to 0°");
  }
  else {
      Serial.println("Unknown color. No movement.");
      return;
  }

  // Move to target color position
  setServoAngle(servoChannel, targetAngle);
  delay(3000);  // Hold position briefly

  // Return to neutral
  Serial.println("Returning to neutral position");
  setServoAngle(servoChannel, neutralpos);
  delay(300);
}