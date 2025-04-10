#include <PPMReader.h>
#include <Servo.h>

///////////ESC control///////////
//Servo ESC1, ESC2, ESC3, ESC4;
Servo ESC1;

///////////Receiver///////////
byte interruptPin = 2;      //Input pin
byte channelAmount = 6;     //Expected channels
PPMReader ppm(interruptPin, channelAmount);

//deadpoints
int minres=1400;
int maxres=1600;

int adv1=1500;           //RC control Channel 2
int deadp=90;            //deadpoint
int speed=90;

///////////Motors///////////
#define motorDel 50

//all pins must be PWM enabled pin with ~ printed beside them
#define M1 8
// #define M2 9
// #define M3 10
// #define M4 11


void setup() {
  //Serial.begin(115200);
  ESC1.attach(M1);  //ESC control
  // ESC2.attach(M2);  //1   3
  // ESC3.attach(M3);  //2   4
  // ESC4.attach(M4);

  ESC1.write(deadp);
  delay(500);
}

void loop() {
  adv1=ppm.latestValidChannelValue(2, 0);     // Read Channel 2 (forward, reverse)
  
  test(adv1);                      //Motors function
}

void test(int signal1){
  if(signal1>minres && signal1<maxres){
    ESC1.write(deadp);
  }
  else if(signal1>maxres){
    speed=map(signal1,1500,2000,95,180);
    ESC1.write(speed);
  }
  else if(signal1<minres){
    speed=map(signal1,1500,1000,0,85);
    ESC1.write(speed);
  }
}