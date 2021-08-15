#include <PIDController.h>

#include "ros.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

#define encA1 2
#define encA2 4
#define motAPWM 5
#define motADir 12

#define encB1 3
#define encB2 6
#define motBPWM 11
#define motBDir 13


volatile long encAPos = 0;
volatile long encBPos = 0;
PIDController posA_pid, posB_pid;
int motAVal = 0;
int motBVal = 0;

float encADiff;
float encBDiff;
float encAPrev;
float encBPrev;
float encAErr;
float encBErr;

long int inputVal = 0;
long int targetPosA = 0;
long int targetPosB = 0;

float demand1;
float demand2;
float demandx;
float demandz;

unsigned long currentMillis;
unsigned long previousMillis;

void velCallback(const geometry_msgs::Twist & Vel)
{
  demandx = vel.linear.x;
  demandz = vel.angular.z;

  demandx = constrain(demandx, -0.25, 0.25);
  demandz = constrain(demandz, -1, 1);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velCallback);

void setup() 
{
  Serial.begin(115200);

  pinMode(encA1, INPUT_PULLUP);
  pinMode(encA2, INPUT_PULLUP);
  pinMode(motAPWM, OUTPUT);
  pinMode(motADir, OUTPUT);

  pinMode(encB1, INPUT_PULLUP);
  pinMode(encB2, INPUT_PULLUP);
  pinMode(motBPWM, OUTPUT);
  pinMode(motBDir, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encA1), encACount, RISING);
  attachInterrupt(digitalPinToInterrupt(encB1), encBCount, RISING);

  encAPos = 0;
  encBPos = 0;

  nh.initNode();
  nh.subscribe(sub);

  posA_pid.begin();
  posA_pid.tune(4, 4, 0.03);
  posA_pid.limit(-200, 200);
  posB_pid.begin();
  posB_pid.tune(4, 4, 0.03);
  posB_pid.limit(-200, 200);
}

void loop()
{
  nh.spinOnce();
  
  currentMillis = millis();

  if(currentMillis - previousMillis >= 10)
  {
    if(Serial.available()>0)
    {
      char c = Serial.read();
      if(c == 'w')
      {
        demandx = 0.5;
        demandz = 0.5;
      }
      else if(c == 's')
      {
        demandx = 0.25;
        demandz = 0;
      }
      else if(c == 'a')
      {
        demandx = 0;
        demandz = -0.25;
      }
      else if(c == 'd')
      {
        demandx = 0;
        demandz = 0.25;
      }
      else if(c == 'x')
      {
        demandx = 0;
        demandz = 0;
      }
    }
  }

  demand1 = demandx - (demandz*0.215);
  demand2 = demandx + (demandz*0.215);

  encADiff = encAPos - encAPrev;
  encBDiff = encBPos - encBPrev;

  encAErr = (demand1*49) - encADiff;
  encBErr = (demand2*49) - encBDiff;

  encAPrev = encAPos;
  encBPrev = encBPos;

  targetPosA = demand1*49;
  inputA = encADiff;
  targetPosB = demand2*49;
  inputB = encBDiff;
  
  //updateInput();

  posA_pid.setpoint(targetPosA);
  posB_pid.setpoint(targetPosB);
  motAVal = posA_pid.compute(inputA);
  motBVal = posB_pid.compute(inputB);

  if(motAVal > 0)
  {
    motAAntiClock(motAVal);
  }
  else
  {
    motAClock(abs(motAVal));
  }
  if(motBVal > 0)
  {
    motBAntiClock(motBVal);
  }
  else
  {
    motBClock(abs(motBVal));
  }

  Serial.print("Encoder count A, B: "); Serial.print(encAPos); Serial.print(", ");
  Serial.print(encBPos);
  Serial.print("\t"); Serial.print("Pos aim: "); Serial.println(targetPos);
  
  delay(10);
}

void encACount()
{
  if(digitalRead(encA2) == HIGH)
  {
    encAPos ++;
  }
  else
  {
    encAPos --;
  }
}
void encBCount()
{
  if(digitalRead(encB2) == HIGH)
  {
    encBPos ++;
  }
  else
  {
    encBPos --;
  }
}

void motAClock(int pwmOut)
{
  if(pwmOut > 60)
  {
    digitalWrite(motADir, false);
    analogWrite(motAPWM, pwmOut);
  }
  else
  {
    digitalWrite(motAPWM, LOW);
  }
}
void motAAntiClock(int pwmOut)
{
  if(pwmOut > 60)
  {
    digitalWrite(motADir, true);
    analogWrite(motAPWM, pwmOut);
  }
  else
  {
    digitalWrite(motAPWM, LOW);
  }
}
void motBClock(int pwmOut)
{
  if(pwmOut > 60)
  {
    digitalWrite(motBDir, true);
    analogWrite(motBPWM, pwmOut);
  }
  else
  {
    digitalWrite(motBPWM, LOW);
  }
}
void motBAntiClock(int pwmOut)
{
  if(pwmOut > 60)
  {
    digitalWrite(motBDir, false);
    analogWrite(motBPWM, pwmOut);
  }
  else
  {
    digitalWrite(motBPWM, LOW);
  }
}

void updateInput()
{
  if(Serial.available())
  {
    inputVal = Serial.parseInt();
    targetPos += (((float)inputVal/360) * 3360);
  }
}
