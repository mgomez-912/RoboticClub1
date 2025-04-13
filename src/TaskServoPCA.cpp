#include "TaskServoPCA.h"

// Initialize the PWM driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void ServoPCA(void *pvParameters) {
    initializeServo();

    while(true) {
      // Sweep the servo back and forth
        Serial.println("Sweeping servo...");
        sweepServo();

      vTaskDelay(taskServoPCA.getIntervalms() / portTICK_PERIOD_MS);  // Delay for the blink time
    }

}

// Function to initialize the PCA9685
// In TaskServoPCA.cpp - modify initializeServo()
void initializeServo() {
  Serial.begin(115200);
  while(!Serial); // Wait for serial monitor in debug mode
  
  Serial.println("\n\nXTARK S620A-180 Servo Test");
  Serial.printf("Using I2C pins: SDA=%d, SCL=%d\n", SDA_PIN, SCL_PIN);

  Wire.begin(SDA_PIN, SCL_PIN);
  if(!pwm.begin()) {
    Serial.println("PCA9685 not found!");
    while(1); // Halt if initialization fails
  }
  
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  Serial.printf("PCA9685 initialized at %dHz\n", SERVO_FREQ);
  
  // Test communication
  pwm.setPWM(SERVO_CHANNEL, 0, 0); // Start with servo off
  delay(1000);
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
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
  Serial.printf("Channel %d set to %d degrees (pulse: %d)\n", channel, angle, pulse);
}

// Function to sweep the servo
void sweepServo() {
  Serial.println("Moving servo from 0 to 180 degrees");
  
  // Move from 0 to 180 degrees
  for (int angle = 0; angle <= 180; angle++) {
    setServoPulse(SERVO_CHANNEL, angle);
    delay(15);
  }
  
  delay(500);
  
  Serial.println("Moving servo from 180 to 0 degrees");
  
  // Move from 180 to 0 degrees
  for (int angle = 180; angle >= 0; angle--) {
    setServoPulse(SERVO_CHANNEL, angle);
    delay(15);
  }
  
  delay(500);
}