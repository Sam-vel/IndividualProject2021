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
PIDController pos_pid;
int motVal = 255;

long int inputVal = 0;
long int targetPos = 0;

void setup() 
{
  Serial.begin(9600);

  pinMode(encA1, INPUT_PULLUP);
  pinMode(encA2, INPUT_PULLUP);
  pinMode(motAPWM, OUTPUT);

  pinMode(encB1, INPUT_PULLUP);
  pinMode(encB2, INPUT_PULLUP);
  pinMode(motBPWM, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encA1), encACount, RISING);
  attachInterrupt(digitalPinToInterrupt(encB1), encBCount, RISING);

  pos_pid.begin();
  pos_pid.tune(20, 0, 2500);
  pos_pid.limit(-255, 255);
}

void loop()
{
  updateInput();

  pos_pid.setpoint(targetPos);
  motVal = pos_pid.compute(encAPos);

  if(motVal > 0)
  {
    motAAntiClock(motVal);
  }
  else
  {
    motAClock(abs(motVal));
  }

  Serial.print("Encoder count A, B: "); Serial.print(encAPos); Serial.print(", ");
  Serial.print(encBPos);
  Serial.print("\t"); Serial.print("Pos aim: "); Serial.println(targetPos);

  //Serial.print(encAPos); Serial.print("\t"); Serial.println(encBPos);
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
  motBClock(pwmOut);
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
  motBAntiClock(pwmOut);
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
