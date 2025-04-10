#include <Servo.h>

Servo myServo;  // Create a servo object

const int servoPin = 9;  // Define the pin the servo is connected to

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Attach the servo on the servoPin
  myServo.attach(servoPin);

  Serial.println("Continuous rotation servo test starting...");

  myServo.write(90);  // 90 corresponds to stop
  delay(1500);
}

void loop() {
  // Rotate clockwise at full speed
  Serial.println("Rotating clockwise at full speed");
  myServo.write(0);   // 0 corresponds to full speed clockwise
  delay(1500);        // Rotate for 2 seconds
  
  // Stop the servo
  Serial.println("Stopping the servo");
  myServo.write(90);  // 90 corresponds to stop
  delay(1500);        // Wait for 2 seconds

  // Rotate counterclockwise at full speed
  Serial.println("Rotating counterclockwise at full speed");
  myServo.write(180); // 180 corresponds to full speed counterclockwise
  delay(1500);        // Rotate for 2 seconds
  
  // Stop the servo
  Serial.println("Stopping the servo");
  myServo.write(90);  // 90 corresponds to stop
  delay(1500);        // Wait for 2 seconds
}