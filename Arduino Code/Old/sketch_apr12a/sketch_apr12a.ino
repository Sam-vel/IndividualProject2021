#include <PIDController.h>

#define encA1 2
#define encA2 4
#define motAPWM 5
#define motADir 12

#define encB1 3
#define encB2 6
#define motBPWM 11
#define motBDir 13

volatile long int encAPos = 0;
volatile long int encBPos = 0;
PIDController posB_pid;
int motAVal = 0;
int motBVal = 0;

long int inputVal = 0;
long int targetPos = 0;

void setup() 
{
  Serial.begin(19200);

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

  posB_pid.begin();
  posB_pid.tune(10, 0, 2500);
  posB_pid.limit(-255, 255);
}

void loop()
{
  updateInput();

  posB_pid.setpoint(targetPos);

  motBVal = posB_pid.compute(encBPos);

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
  
  delay(2);
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
  if(pwmOut > 40)
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
  if(pwmOut > 40)
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
  if(pwmOut > 40)
  {
    digitalWrite(motBDir, false);
    analogWrite(motBPWM, pwmOut);
  }
  else
  {
    digitalWrite(motBPWM, LOW);
  }
}
void motBAntiClock(int pwmOut)
{
  if(pwmOut > 40)
  {
    digitalWrite(motBDir, true);
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
