#include <PIDController.h>

#define encoderA 2
#define encoderB 5
#define motorA 3

#define potPin A4

const int numReadings = 15;

int readings[numReadings];
int readIndex = 0;
int total = 0;
int average = 0;
long int aimPos = 0;

long int inputVal = 0;
long int targetPos = 0;

volatile long int encoder_pos = 0;
PIDController pos_pid;
int motor_value = 255;

void setup() {
  Serial.begin(9600);
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);
  pinMode(motorA, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2), encoder, RISING);

  pinMode(potPin, INPUT_PULLUP);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  pos_pid.begin();    
  pos_pid.tune(20, 0, 2500);    
  pos_pid.limit(-255, 255);
}

void loop() 
{
  updateInput();
  //potSmooth();
  
  pos_pid.setpoint(targetPos);
  motor_value = pos_pid.compute(encoder_pos);

  if (motor_value > 0) {
    MotorCounterClockwise(motor_value);
  } else {
    MotorClockwise(abs(motor_value));
  }

  Serial.print("Encoder Pos: "); Serial.print(encoder_pos); 
  Serial.print("\t"); Serial.print("Pos Aim: "); Serial.println(targetPos);
  delay(2);
}

void encoder() {
  if (digitalRead(encoderB) == HIGH) {
    encoder_pos++;
  } else {
    encoder_pos--;
  }
}

void MotorClockwise(int power){
  if (power > 20) {
    digitalWrite(12, false);
    analogWrite(motorA, power);
  } else {
    digitalWrite(motorA, LOW);
  }
}
void MotorCounterClockwise(int power) {
  if (power > 20) {
    digitalWrite(12, true);
    analogWrite(motorA, power);
  } else {
    digitalWrite(motorA, LOW);
  }
}

void potSmooth()
{
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(potPin);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  aimPos = map(average, 0, 1023, -1725, 1725);
  delay(1);        // delay in between reads for stability
}
void updateInput()
{
  if(Serial.available())
  {
    inputVal = Serial.parseInt();
    targetPos += (((float)inputVal/360) * 3360);
  }
}
