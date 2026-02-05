// 8-Channel IR sensor pins
int irPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Motor driver control pins
int IN1 = 5;
int IN2 = 6;
int EN  = 9;
-
void setup()
{
  // IR sensors as input
  for (int i = 0; i < 8; i++)
  {
    pinMode(irPins[i], INPUT);
  }

  // Motor pins as output
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN, OUTPUT);

  // Motor OFF initially
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(EN, LOW);
}

void loop()
{
  int blackDetected = 0;

  // Check all 8 IR sensors
  for (int i = 0; i < 8; i++)
  {
    if (digitalRead(irPins[i]) == LOW) // BLACK detected
    {
      blackDetected = 1;
      break;
    }
  }

  if (blackDetected)
  {
    // Motor ON
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(EN, 200);   // speed control (0–255)
  }
  else
  {
    // Motor OFF
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(EN, 0);
  }
}
