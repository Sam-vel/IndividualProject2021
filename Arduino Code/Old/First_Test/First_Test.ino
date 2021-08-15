#define PWMpin 3
#define potInput A4

#define encoder0pinA 2
#define encoder0pinB 4
#define fullRev 13440

int potMapped = 0;
long duration = 0;
bool Direction = false;
bool encoder0PinALast = false;



void setup() {
  pinMode(PWMpin, OUTPUT);

 Serial.begin(115200);//Initialize the serial port
 EncoderInit();//Initialize the module
}

void loop() {
  potMapped = map(analogRead(potInput), 0, 1023, 0, 255);
  analogWrite(PWMpin, potMapped);

  //Serial.println(digitalRead(encoder0pinA));
  Serial.println(digitalRead(encoder0pinB));

 //Serial.print("Pulse:");
 //Serial.println(duration);
 duration = 0;
 delay(100);
}

void EncoderInit() {

 Direction = true;//default -> Forward  
 pinMode(encoder0pinB,INPUT);  
 attachInterrupt(0, wheelSpeed, CHANGE);
}

void wheelSpeed() {

 int Lstate = digitalRead(encoder0pinA);
 if((encoder0PinALast == LOW) && Lstate==HIGH)
 {
   int val = digitalRead(encoder0pinB);
   if(val == LOW && Direction)
   {
     Direction = false; //Reverse
   }
   else if(val == HIGH && !Direction)
   {
     Direction = true;  //Forward
   }
 }
 encoder0PinALast = Lstate;

 if(!Direction)  duration++;
 else  duration--;
}
