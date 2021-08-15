// Motor encoder output pulse per rotation (change as required)
#define encCountPerRev 3603.2

// Encoder output to Arduino Interrupt pin
#define encIn 2 

// MD10C PWM connected to pin 10
#define pwmPin 3
// MD10C DIR connected to pin 12
#define dirPin 12

// Analog pin for potentiometer
#define potInput A4

// Pulse count from encoder
volatile float encVal = 0;

// One-second interval for measurements
int interval = 1000;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

// Variable for RPM measuerment
long rpm = 0;

// Variable for PWM motor speed output
int motPWM = 0;

float degTurn = 0;
int degToSpin = 0;

void setup()
{
  // Setup Serial Monitor
  Serial.begin(57600); 
  
  // Set encoder as input with internal pullup  
  pinMode(encIn, INPUT_PULLUP); 

  // Set PWM and DIR connections as outputs
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, true);
  
  // Attach interrupt 
  attachInterrupt(digitalPinToInterrupt(encIn), updateEncoder, RISING);
  
  // Setup initial values for timer
  previousMillis = millis();
}

void loop()
{ 
  // Control motor with potentiometer
  //motPWM = map(analogRead(potInput), 0, 1023, -255, 255);
  degToSpin = Serial.parseInt();

  if(degTurn < degToSpin)
  {
    motPWM = 150;
  }
  else
  {
    motPWM = 0;
  }
  
  // Write PWM to controller
  if(motPWM >=0)
  {
    digitalWrite(dirPin, true);
    analogWrite(pwmPin, motPWM);
  }
  else if(motPWM < 0)
  {
    digitalWrite(dirPin, false);
    analogWrite(pwmPin, abs(motPWM));
  }

  degTurn = ((encVal / encCountPerRev) * 360);
  
  // Update RPM value every second
  currentMillis = millis();
  if(currentMillis - previousMillis > interval) 
  {
    previousMillis = currentMillis;


    // Calculate RPM
    //rpm = (float)(encVal * 600 / encCountPerRev);

    Serial.print("PWM VALUE: ");
    Serial.print(motPWM);
    Serial.print('\t');
    Serial.print(" PULSES: ");
    Serial.print(encVal);
    Serial.print('\t');
    Serial.print(" Degrees turned: ");
    Serial.print(degTurn);
    Serial.println(" degrees");
  }
}

void updateEncoder()
{
  // Increment value for each pulse from encoder
  encVal++;
}
