///////////Line sens///////////
#define Sens1 22
#define Sens2 23
#define Sens3 24
#define Sens4 25
#define Sens5 26
#define Sens6 27
#define Sens7 28
#define Sens8 29

int c = 0;
int posval = 0;

int sensorBuffer[8]={0, 0, 0, 0, 0, 0, 0, 0}; 

///////////PID///////////
float Kp = 0.02; // related to the proportional control term;
float Ki = 0.000; // related to the integral control term;
float Kd = 0.10; // related to the derivative control term;
int P;
int I;
int D;

int lastError = 0;

const uint8_t maxspeeda = 240;
const uint8_t maxspeedb = 240;
const uint8_t basespeeda = 65;
const uint8_t basespeedb = 65;

int PID_left=0;
int PID_right=1;
int PID_dir=2;

///////////Motors///////////
#define delmotors 50
#define lim 255 // speed limit

// all pins must be PWM enabled pin with ~ printed beside them
#define M1_Forward 8
#define M1_Reverse 9
#define M2_Forward 10
#define M2_Reverse 11

bool CW = true;
bool CCW = false;

///////////Ultrasound///////////
#include <NewPing.h>

#define TRIGGER_PIN  6  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     7 // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 250 // Maximum distance we want to measure (in centimeters).

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

///////////Arm///////////
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define MIN_PULSE_WIDTH 650
#define MAX_PULSE_WIDTH 2350
#define DEFAULT_PULSE_WIDTH 1500
#define FREQUENCY 50

Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);

//arm positions
#define open 15
#define close 100
#define frontElb 15
#define backElb 210
#define liftElb 50
#define center 50     //old one 55

#define pause 3000        //variable delay


///////////counter///////////
// int exitC=0;        //c
// int turnC=0;        //f
// bool pick=false;

int dist=20;
int distance = 0;


void setup() {
  Serial.begin(9600);

///////////arm///////////
  servos.begin();  
  servos.setPWMFreq(50); //Frecuecia PWM de 50Hz o T=16,66ms

  ///////////Line sens///////////
  pinMode(Sens1, INPUT);
  pinMode(Sens2, INPUT);
  pinMode(Sens3, INPUT);
  pinMode(Sens4, INPUT);
  pinMode(Sens5, INPUT);
  pinMode(Sens6, INPUT);
  pinMode(Sens7, INPUT);
  pinMode(Sens8, INPUT);

  Serial.println("Ultrasonic sensor test starting...");
  distance = sonar.ping_cm();

  delay(500);
  armpos(center, backElb, open, pause,50);
  armpos(center, backElb, close, 500,50);
  forward(0, CW, 0, CW);
}

void loop() {
  distance = sonar.ping_cm();       
  delay(30);        // Wait for a short while before the next measurement
  Serial.println(distance);
  navigation();
}

///////////Navigation///////////
void navigation(){
  switch (c) {
    case 0:
      if(distance>dist){           //distance to put the ore should be around 5cm
      PID_Control(CW, PID_dir);
      }
      else if(distance<=dist){
        fullBrake(500);
        armpos(center, backElb, close, 500,50);
        armpos(center, 110, close, 1000,50);
        armpos(center, frontElb, close, 2000,50);
        armpos(center, frontElb, open, pause,50);
        c = 1;
      }
      break;
    case 1:                         //drop first ore
      while (distance<40){
      forward(150, CCW, 150, CCW);
      distance = sonar.ping_cm();       
      delay(30); 
      }
      fullBrake(500);
      forward(150, CW, 100, CCW);
      delay(500);
      fullBrake(500);
      c=2;
      armpos(center, backElb, close, pause,50);
      break;
    case 2:                       //find second line and position to get 2nd ore
      PID_Control(CW, PID_right);
      if(distance<=25){
        c=3;
        fullBrake(500);
      }
      break;
    case 3:                     
      forward(100, CW, 50, CCW);
      delay(300);
      fullBrake(500);
      forward(75, CCW, 75, CCW);
      delay(300);
      fullBrake(500);
      armpos(center, frontElb+90, open, 3000,0); 
      armpos(center, frontElb+90, close+10, 3000,0);
      armpos(center, frontElb+130, close+10, 2000,0);
      c=4;
      break;
    case 4:
      forward(150, CCW, 150, CCW);
      delay(600);
      fullBrake(500);
      forward(150, CW, 100, CCW);
      delay(500);
      fullBrake(500);
      c=5;
      break;
    case 5:
      forward(150,CW,150,CW);
      delay(900);
      fullBrake(500);
      //drop first ore
      armpos(center, frontElb+100, close+5, 1000,80);
      armpos(center, frontElb, close+5, 2000,80);
      armpos(center, frontElb, open, 2000,50);
      c=6;
      break;
    case 6:
      forward(150, CCW, 150, CCW);
      delay(400);
      fullBrake(500);
      c=7;
      break;
    // case 3:
    //     armpos(center+48, frontElb, open, 3000);
    //     armpos(center+48, frontElb, close, 2000);
    //     armpos(center+48, frontElb+70, close, 1000);
    //     armpos(center, backElb, close, 2000);
    //     armpos(center, backElb, open, 1000);
    //     armpos(center, backElb-40, open, 1000);
    //     c=4;
    //   break;
    // case 4:
    //     armpos(center-30, frontElb, open, 3000);
    //     armpos(center-30, frontElb, close, 2000);
    //     armpos(center-30, frontElb+80, close, 1000);
    //     armpos(center, frontElb+80, close, 2000);
    //     c=5;
    //   break;
    // case 5:
    //     forward(150, CCW, 150, CCW);
    //     delay(600);
    //     fullBrake(500);
    //     forward(150, CW, 100, CCW);
    //     delay(500);
    //     fullBrake(500);
    //     c=6;
    //   break;
    // case 6:
    //     forward(150,CW,150,CW);
    //     delay(900);
    //     fullBrake(500);
    //     //drop first ore
    //     armpos(center, frontElb+80, close, 2000);
    //     armpos(center, frontElb, close, 2000);
    //     armpos(center, frontElb, open, 2000);
    //     //pick second ore
    //     armpos(center, backElb, open, 2000);
    //     armpos(center, backElb, close, 2000);
    //     c=7;
    //   break;
    // case 7:
    //     //reverse and get position to drop the second ore
    //     forward(150, CCW, 150, CCW);
    //     delay(400);
    //     fullBrake(500);
    //     forward(150, CW, 100, CCW);
    //     delay(300);
    //     fullBrake(500);
    //     forward(150, CW, 150, CW);
    //     delay(600);
    //     fullBrake(500);
    //     c=8;
    //   break;
    // case 8:
    //     //drop second ore
    //     armpos(center, backElb, close, 2000);
    //     armpos(center, frontElb+80, close, 2000);
    //     armpos(center, frontElb, close, 2000);
    //     armpos(center, frontElb, open, 2000);
    //     forward(150, CCW, 150, CCW);
    //     delay(600);
    //     fullBrake(500);
    //     c=9;
    //   break;
      
    default:
      forward(0, CW, 0, CW);
      break;
      }
  delay(1);
}

///////////Arm///////////
int pulseWidth(int angle){
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  //Serial.println(analog_value);
  return analog_value;
}

void armpos(int angle, int elbow, int claw, int del, int wrist){
  servos.setPWM(0, 0, pulseWidth(claw)); // CLOSE CLAW
  servos.setPWM(1, 0, pulseWidth(0)); // ? IS LOWER POSITION
  servos.setPWM(2, 0, pulseWidth(wrist)); // 45 IS LOWER POSITION
  servos.setPWM(3, 0, pulseWidth(elbow)); // DO TAKE HAND // ORI 5
  servos.setPWM(4, 0, pulseWidth(115)); //  fiX PRO NUMBER 4
  servos.setPWM(5, 0, pulseWidth(angle)); //  final postion
  delay(del);
}


///////////Line sens///////////
int readLineBlack() {
  int sensorValues[8];
  for (int i = 0; i < 8; i++) {
    if(digitalRead(22+i)==0){sensorValues[i]=1;}
    else{sensorValues[i]=0;}
  }
  float position = 0;
  int c = 0;
  for (int i = 0; i < 8; i++) {
    position += (i) * sensorValues[i];
    if (sensorValues[i] == 1) c++;
  }
  if(c==0){position=0;}
  else{position /= c;}
  //Serial.println(position*2000);
  return position * 2000;
}

int filterLineBlack() {
  int sensorValues[8];
  int filterValues[8];
  for (int i = 0; i < 8; i++) {
    sensorValues[i]=digitalRead(22+i);
    filterValues[i]=sensorValues[i]+sensorBuffer[i];
    sensorBuffer[i]=sensorValues[i];
    if(filterValues[i]>0){filterValues[i]=0;}
    else{filterValues[i]=1;}
  }
  float position = 0;
  int c = 0;
  for (int i = 0; i < 8; i++) {
    position += (i) * filterValues[i];
    if (filterValues[i] == 1) c++;
  }
  if(c==0){position=0;}
  else{position /= c;}
  //Serial.println(position*2000);
  return position * 2000;
}


///////////Motors///////////
void forward(int speeda, bool dira, int speedb, bool dirb) {
  M1(dira, speeda);
  M2(dirb, speedb);
}

void M1(bool direction, int speed) {
  if (speed < 0 || speed > 255) {
    return;
  }
  if (direction == CW) {
    analogWrite(M1_Forward, speed);
    analogWrite(M1_Reverse, LOW);
  } else {
    analogWrite(M1_Reverse, speed);
    analogWrite(M1_Forward, LOW);
  }
}

void M2(bool direction, int speed) {
  if (speed < 0 || speed > 255) {
    return;
  }
  if (direction == CW) {
    analogWrite(M2_Forward, speed);
    analogWrite(M2_Reverse, LOW);
  } else {
    analogWrite(M2_Reverse, speed);
    analogWrite(M2_Forward, LOW);
  }
}

void brake(int motor) {
  if (motor == 1) {
    analogWrite(M1_Forward, 1);
    analogWrite(M1_Reverse, 1);
  } else {
    analogWrite(M2_Forward, 1);
    analogWrite(M2_Reverse, 1);
  }
}

void fullBrake(int del) {
  brake(1);
  brake(2);
  delay(del);
}

void PID_Control(bool direction, int RLdir) {
  // uint16_t position = readLineBlack();
  uint16_t position = filterLineBlack();
  if (position == 0 && RLdir==1) { // Assuming 0 means no line detected, adjust this condition if necessary
      forward(50, !direction, 0, direction); // Rotate CW (Right)
    return;
  }
  if (position == 0 && RLdir==0) { // Assuming 0 means no line detected, adjust this condition if necessary
      forward(0, direction, 100, !direction); // Rotate CCW (Left)
    return;
  }
  int error = 7000 - position;
  //Serial.println(position);
  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P * Kp + I * Ki + D * Kd;

  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;

  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  }

  forward(motorspeeda, direction, motorspeedb, direction);
}