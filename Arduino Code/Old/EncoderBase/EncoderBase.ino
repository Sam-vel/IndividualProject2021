#define rotInA 2
#define rotInB 4

long counter = 0;
int currentStateA;
int previousStateA;

String encDir = "";

void setup()
{
  pinMode(rotInA, INPUT_PULLUP);
  pinMode(rotInB, INPUT_PULLUP);

  Serial.begin(9600);

  previousStateA = digitalRead(rotInA);
}

void loop()
{
  
  
  currentStateA = digitalRead(rotInA);

  if(currentStateA != previousStateA)
  {
    if(digitalRead(rotInB) != currentStateA)
    {
      counter --;
      encDir = "CCW";
    }
    else
    {
      counter ++;
      encDir = "CW";
    }

    Serial.print("Direction: ");
    Serial.print(encDir);
    Serial.print("/t");
    Serial.print("Value: ");
    Serial.print(counter);
  }

  previousStateA = currentStateA;
}
