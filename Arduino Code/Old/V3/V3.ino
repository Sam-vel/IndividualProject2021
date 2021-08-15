#include <PIDController.h>

#define encA1 2
#define encA2 5
#define motAPWM 3
#define motDir 12

#define potPin A4

volatile long int encPos = 0;
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

  attachInterrupt(digitalPinToInterrupt(encA1), encCount, RISING);

  pos_pid.begin();
  pos_pid.tune(20, 0, 2500);
  pos_pid.limit(-255, 255);
}

void loop()
{
  updateInput();

  pos_pid.setpoint(targetPos);
  motVal = pos_pid.compute(encPos);

  if(motVal > 0)
  {
    motAntiClock(motVal);
  }
  else
  {
    motClock(abs(motVal));
  }

  Serial.print("Encoder count: "); Serial.print(encPos);
  Serial.print("\t"); Serial.print("Pos aim: "); Serial.println(targetPos);
  delay(2);
}

void encCount()
{
  if(digitalRead(encA2) == HIGH)
  {
    encPos ++;
  }
  else
  {
    encPos --;
  }
}

void motClock(int pwmOut)
{
  if(pwmOut > 40)
  {
    digitalWrite(motDir, false);
    analogWrite(motAPWM, pwmOut);
  }
  else
  {
    digitalWrite(motAPWM, LOW);
  }
}
void motAntiClock(int pwmOut)
{
  if(pwmOut > 40)
  {
    digitalWrite(motDir, true);
    analogWrite(motAPWM, pwmOut);
  }
  else
  {
    digitalWrite(motAPWM, LOW);
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
