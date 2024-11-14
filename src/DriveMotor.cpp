#include "DriveMotor.h"

const int lim = 150;         //speed limit
const int motorDel = 50;

//all pins must be PWM enabled pin with ~ printed beside them
const int M1_Forward = 8;
const int M1_Reverse = 9;
const int M2_Forward = 10;
const int M2_Reverse = 11;

int c=0;                //decision cases
int baseSpeed = 0;

//direction states
bool CW = true;
bool CCW = false;
bool lastDirM1;
bool lastDirM2;

//deadpoints
int minres=1400;
int maxres=1600;

//channels
int adv1=1500;           //RC control Channel 2
int turn1=1500;          //RC control Channel 4

void MotorDriving(void *pvParameters){
    pinMode(M1_Forward, OUTPUT);
    pinMode(M1_Reverse, OUTPUT);
    pinMode(M2_Forward, OUTPUT);
    pinMode(M2_Reverse, OUTPUT);

    while(true){
        adv1 = channelValues[2];         // Read Channel 2 (forward, reverse)
        turn1 = channelValues[4];       // Read Channel 4 (turn right, left)

        decision(adv1, turn1);                      //Motors function

        vTaskDelay(taskRGBLed.getIntervalms() / portTICK_PERIOD_MS);
    }
}

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

void turn(bool dir1, bool dir2, int speed1, int speed2){
  M1(dir1, speed1, 1);
  M1(dir2, speed2, 2);
}

void M1(bool direction,int speed, bool motorNumber)
{
    if(motorNumber == 1){
        if(speed<0 || speed>255){
            vTaskDelay(1);
        }
        else if(direction != lastDirM1){
            stop(3,500);
        }
        else if(direction == CCW)
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
    if(motorNumber == 1){
        if(speed<0 || speed>255){
            vTaskDelay(1);
        }
        else if(direction != lastDirM2){
            stop(3,500);
        }
        else if(direction == CW)
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
  }else if(motor == 2){
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