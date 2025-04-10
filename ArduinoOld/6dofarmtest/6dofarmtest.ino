#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MIN_PULSE_WIDTH 650
#define MAX_PULSE_WIDTH 2350
#define DEFAULT_PULSE_WIDTH 1500
#define FREQUENCY 50

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

////////Servos//////////
#define open 0
#define close 100
#define frontElb 0
#define backElb 190
#define liftElb 50
#define center 55

#define pause 4000

void setup() {
  servos.begin();  
  servos.setPWMFreq(50); //Frecuecia PWM de 50Hz o T=16,66ms
  Serial.begin(9600);
}

int pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  Serial.println(analog_value);
  return analog_value;
}

void loop() {
  armpos(center, frontElb, open, pause);
  armpos(center, frontElb, close, pause);
  armpos(center, liftElb, close, pause);
  armpos(center, backElb, close, pause);
  armpos(center, backElb, open, pause);
}

void armpos(int angle, int elbow, int claw, int del){
  servos.setPWM(0, 0, pulseWidth(claw)); // CLOSE CLAW
  servos.setPWM(1, 0, pulseWidth(0)); // ? IS LOWER POSITION
  servos.setPWM(2, 0, pulseWidth(45)); // 45 IS LOWER POSITION
  servos.setPWM(3, 0, pulseWidth(elbow)); // DO TAKE HAND // ORI 5
  servos.setPWM(4, 0, pulseWidth(105)); //  fiX PRO NUMBER 4
  servos.setPWM(5, 0, pulseWidth(angle)); //  final postion
  delay(del);
}