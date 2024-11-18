#include "PWMExtender.h"

void PWMExtender(int Channel, int val){
// void PWMExtender(void *pvParamenters){
    Adafruit_PWMServoDriver drivePWM = Adafruit_PWMServoDriver(0x40);
    Wire.begin(17,18);                          //SDA,SCL
    drivePWM.begin();  
    drivePWM.setPWMFreq(FREQUENCY); //PWM freq 50Hz or T=16,66ms

    // while(true){
    //     drivePWM.setPWM(0, 0, pulseWidth(channelValues[1]));
    //     drivePWM.setPWM(1, 0, pulseWidth(channelValues[2])); 
    //     Serial.println(channelValues[1]);
        
    //     vTaskDelay(taskPWMExt.getIntervalms() / portTICK_PERIOD_MS);
    // }
    drivePWM.setPWM(Channel, 0, pulseWidth(val));
}

int pulseWidth(int angle){
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 1000, 2000, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  //Serial.println(analog_value);
  return analog_value;
}