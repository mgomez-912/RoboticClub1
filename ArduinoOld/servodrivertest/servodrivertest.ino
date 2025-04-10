#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PPMReader.h>

///////////Receiver///////////
byte interruptPin = 2;      //Input pin
byte channelAmount = 6;     //Expected channels
PPMReader ppm(interruptPin, channelAmount);

//deadpoints
int minres=1400;
int maxres=1600;

int adv1=1500;           //RC control Channel 2
int turn1=1500;          //RC control Channel 4
int pos=1500;            //SWB position Channel 5

///////////PWM DRIVER///////////
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);

// unsigned int pos0=172; // ancho de pulso en cuentas para pocicion 0° (90 optional)
// unsigned int pos90=369; // ancho de pulso en cuentas para pocicion 90°
// unsigned int pos180=565; // ancho de pulso en cuentas para la pocicion 180° (600 optional)

///////////Claw///////////
// unsigned int closepos=310;       //black ones
// unsigned int openpos=150;

unsigned int closepos=580;           //TBS-K20
unsigned int openpos=470;

int n=15;

void setup() {
  servos.begin();  
  servos.setPWMFreq(50); //Frecuecia PWM de 50Hz o T=16,66ms
}


void loop() {
  for (int duty = openpos; duty < closepos; duty=duty+10) {
      servos.setPWM(n,0,duty);
      servos.setPWM(14,0,duty);
  }
  delay(1000);
  for (int duty = closepos; duty > openpos; duty=duty-10) {
      servos.setPWM(n,0,duty);
      servos.setPWM(14,0,duty);
  }
  delay(2000);
}
