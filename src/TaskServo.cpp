#include "TaskServo.h"

void Servot1(void *pvParameters) {
    Servo myservo;

    int pos = 0;        // variable to store the servo position
    int servoPin = 4;  

    // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(50);    // standard 50 hz servo
	myservo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
	// for an accurate 0 to 180 sweep
    
    while(true){
        // for (pos = 105; pos >= 30; pos -= 5) { // goes from 180 degrees to 0 degrees
        //     myservo.write(pos);    // tell servo to go to position in variable 'pos'
        //     delay(150);             // waits 15ms for the servo to reach the position
        // }
        // for (pos = 30; pos <= 105; pos += 5) { // goes from 0 degrees to 180 degrees
		// // in steps of 1 degree
		// myservo.write(pos);    // tell servo to go to position in variable 'pos'
		// delay(150);             // waits 15ms for the servo to reach the position
        // }
        myservo.write(95);
        delay(1000);
        myservo.write(35);
        delay(1000);

        vTaskDelay(taskServot1.getIntervalms() / portTICK_PERIOD_MS);  // Delay for the blink time
    }

}