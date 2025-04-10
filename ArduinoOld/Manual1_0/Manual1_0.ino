#include <PPMReader.h>
#include <Servo.h>

///////////Receiver///////////
byte interruptPin = 2;      //Input pin
byte channelAmount = 6;     //Expected channels
PPMReader ppm(interruptPin, channelAmount);

//deadpoints
int minres=1400;
int maxres=1600;

int adv1=1500;           //RC control Channel 2
int turn1=1500;          //RC control Channel 4
int pos=1500;            //RC control Channel 5


///////////Motors///////////
#define lim 150       //speed limit
#define motorDel 50

//all pins must be PWM enabled pin with ~ printed beside them
#define M1_Forward 8
#define M1_Reverse 9
#define M2_Forward 10
#define M2_Reverse 11

int c=0;          //decision cases
int baseSpeed = 0;

//direction states
bool CW = true;
bool CCW = false;
bool lastDirM1;
bool lastDirM2;

///////////Servo///////////
#define servoPin 6
Servo myServo;

int servoVal=1500;
int prevVal=0;

///////////Claw///////////
#define clawPin 7
Servo clawServo;
int clawVal=1100;
int clawValprev=1500;

///////////Pulley///////////
#define Pu_Forward 4
#define Pu_Reverse 5
bool lastDirPu;


void setup() {
  Serial.begin(115200);
  pinMode(M1_Forward, OUTPUT);      //Motors
  pinMode(M1_Reverse, OUTPUT);
  pinMode(M2_Forward, OUTPUT);
  pinMode(M2_Reverse, OUTPUT);

  myServo.attach(servoPin);         //Servo to pin 6
  myServo.write(0);

  clawServo.attach(clawPin);        // Claw to pin 7
  clawServo.write(0);

  pinMode(Pu_Forward, OUTPUT);      //Pulley
  pinMode(Pu_Reverse, OUTPUT);

  servo(1470);                      //initial value
  claw(1100);

  delay(500);

  stop(3, motorDel);
}

void loop() {
  adv1=ppm.latestValidChannelValue(2, 0);     // Read Channel 2 (forward, reverse)
  turn1=ppm.latestValidChannelValue(4, 0);    // Read Channel 4 (turn right, left)
  pos=ppm.latestValidChannelValue(5, 0);     // Read Channel 5 (up-X-down)
  servoVal = ppm.latestValidChannelValue(3, 0);
  clawVal = ppm.latestValidChannelValue(6, 0);
  
  decision(adv1, turn1);                      //Motors function

  if(pos<1250){                   //Pulley
    Pulley(CW,200);           // Going up
  }
  else if(pos>1750){          // Going down
    Pulley(CCW,50);
  }
  else{
    analogWrite(Pu_Forward,LOW);
    analogWrite(Pu_Reverse,LOW);
  }

  Serial.println(servoVal);
  if(abs(servoVal-prevVal)>50){               //Servo function
    servo(servoVal);
  }
  //Serial.println(clawVal);
  if(abs(clawVal-clawValprev)>100){           //claw function
    claw(clawVal);
  }
}

///////////Claw functions///////////
void claw(int pulsewidth) {
  clawValprev=pulsewidth;
  pulsewidth=map(pulsewidth, 1100, 1900, 90, 150);
  clawServo.write(pulsewidth);
}

///////////Servo functions///////////
void servo(int angle){
    prevVal=angle;
    int servoAngle = map(angle, 1470, 2000, 20, 200); // Map PPM value to servo angle (0-180 degrees)
    myServo.write(servoAngle); // Set servo to the calculated angle
}

///////////Motor functions///////////
void decision(int signal1, int signal2){
  if(signal1>minres && signal1<maxres){
    c=1;
    if(signal2<minres || signal2>maxres){
      c=4;
    }
  }
  else if(signal1>maxres){
    c=2;
  }
  else{
    c=3;
  }

  switch (c) {
  case 1:
      baseSpeed = 0;
      brake(motorDel);
    break;
  case 2:
      baseSpeed = map(signal1,1500,2000,0,lim);
      forward(baseSpeed,signal2);
    break;
  case 3:
    baseSpeed = map(signal1,1450,1000,0,lim); 
      reverse(baseSpeed,signal2);
    break;
  case 4:
      if(signal2<minres){
        baseSpeed=map(signal2,1450,1000,0,lim);
        turn(CW, CCW, baseSpeed, baseSpeed);
      }
      else if(signal2>maxres){
        baseSpeed=map(signal2,1550,1995,0,lim);
        turn(CCW, CW, baseSpeed, baseSpeed);
      }
    break;
  default:
    stop(3,50);
    break;
  }
}

void forward(int speed, int signal2){
  if(signal2>minres && signal2<maxres){
    turn(CW, CW, speed, speed);
    }
    else if(signal2<minres){
      int turnL = (speed)-(map(signal2,1450,1000,0,lim)*0.75);
      if(turnL>0 && turnL<lim){
        turn(CW, CW, speed, turnL);
      }
    }
    else if(signal2>maxres){
      int turnR = (speed)-(map(signal2,1550,1995,0,lim)*0.75);
      if(turnR>0 && turnR<lim){
        turn(CW, CW, turnR, speed);
      }
    }
}


void reverse(int speed, int signal2){
  if(signal2>minres && signal2<maxres){
    turn(CCW, CCW, speed, speed);
  }
  else if(signal2<minres){
    int turnL = (speed)-(map(signal2,1450,1000,0,lim)/2);
    if(turnL>0 && turnL<lim){
      turn(CCW, CCW, speed, turnL);
    }
  }
  else if(signal2>maxres){
    int turnR = (speed)-(map(signal2,1550,1995,0,lim)/2);
    if(turnR>0 && turnR<lim){
      turn(CCW, CCW, turnR, speed);
    }
  }
}

void FRmove(bool dir, int speed, int signal2){
  if(signal2>minres && signal2<maxres){
    turn(dir, dir, speed, speed);
    }
    else if(signal2<minres){
      int turnL = (speed)-(map(signal2,1450,1000,0,lim)/2);
      if(turnL>0 && turnL<2000){
        turn(dir, dir, speed, turnL);
      }
    }
    else if(signal2>maxres){
      int turnR = (speed)-(map(signal2,1550,2000,0,lim)/2);
      if(turnR>0 && turnR<2000){
        turn(dir, dir, turnR, speed);
      }
    }
}

void turn(bool dir1, bool dir2, int speed1, int speed2){
  M1(dir1, speed1);
  M2(dir2, speed2);
}

void M1(bool direction,int speed)
{
  if(speed<0 || speed>255){
    return;
  }
  if(direction != lastDirM1){
    stop(3,500);
  }
  if(direction == CCW)
  {
   analogWrite(M1_Forward,speed);
   analogWrite(M1_Reverse,LOW);
   lastDirM1=CCW;    
  }else{
   analogWrite(M1_Reverse,speed);
   analogWrite(M1_Forward,LOW);  
   lastDirM1=CW;    
  }
}


void M2(bool direction,int speed)
{
  if(speed<0 || speed>255){
    return;
  }
  if(direction != lastDirM2){
    stop(3,500);
  }
  if(direction == CW)
  {
   analogWrite(M2_Forward,speed);
   analogWrite(M2_Reverse,LOW);
   lastDirM2=CW;   
  }else{
   analogWrite(M2_Reverse,speed);
   analogWrite(M2_Forward,LOW); 
   lastDirM2=CCW;    
  }   
}


void brake(int motor)
{
   if(motor == 1)
  {
    analogWrite(M1_Forward,HIGH);
    analogWrite(M1_Reverse,HIGH);   
  }else if(motor == 2){
    analogWrite(M2_Forward,HIGH);
    analogWrite(M2_Reverse,HIGH);     
  } else{
    analogWrite(M1_Forward,HIGH);
    analogWrite(M1_Reverse,HIGH);
    analogWrite(M2_Forward,HIGH);
    analogWrite(M2_Reverse,HIGH);  
  }
}


void stop(int motor, int del)
{
   if(motor == 1)
  {
   analogWrite(M1_Forward,LOW);
   analogWrite(M1_Reverse,LOW);  
  }else if(motor==2){
   analogWrite(M2_Forward,LOW);
   analogWrite(M2_Reverse,LOW);  
  } else{
    analogWrite(M1_Forward,LOW);
    analogWrite(M1_Reverse,LOW);
    analogWrite(M2_Forward,LOW);
    analogWrite(M2_Reverse,LOW);
  }
  delay(del);
}

///////////Pulley function///////////
void Pulley(bool direction,int speed){
  if(speed<0 || speed>255){
    return;
  }
  if(direction != lastDirPu){
    analogWrite(Pu_Forward,LOW);
    analogWrite(Pu_Reverse,LOW); 
    delay(500);
  }
  if(direction == CW){
   analogWrite(Pu_Forward,speed);
   analogWrite(Pu_Reverse,LOW);  
   lastDirPu=CW; 
  }else{
   analogWrite(Pu_Reverse,speed);
   analogWrite(Pu_Forward,LOW); 
   lastDirPu=CCW;   
  }   
}


