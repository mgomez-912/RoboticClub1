#include "PWMExtender.h"

void PWMExtender(void *pvParameters){
    Adafruit_PWMServoDriver drivePWM = Adafruit_PWMServoDriver(0x40);
    drivePWM.begin();  
    drivePWM.setPWMFreq(FREQUENCY); //PWM freq 50Hz or T=16,66ms

    while(true){
        drivePWM.setPWM(0, 0, pulseWidth(channelValues[2])); // CLOSE CLAW
        // servos.setPWM(1, 0, pulseWidth(0)); // ? IS LOWER POSITION
        // servos.setPWM(2, 0, pulseWidth(wrist)); // 45 IS LOWER POSITION
        // servos.setPWM(3, 0, pulseWidth(elbow)); // DO TAKE HAND // ORI 5
        // servos.setPWM(4, 0, pulseWidth(115)); //  fiX PRO NUMBER 4
        // servos.setPWM(5, 0, pulseWidth(angle)); //  final postion
        
        vTaskDelay(taskPWMExt.getIntervalms() / portTICK_PERIOD_MS);
    }
}

int pulseWidth(int angle){
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 1000, 2000, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  //Serial.println(analog_value);
  return analog_value;
}