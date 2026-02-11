#define IR 2

#define STBY 8
#define AIN1 6
#define AIN2 7
#define PWMA 5

void setup() {
  pinMode(IR, INPUT);

  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  digitalWrite(STBY, HIGH);   // Enable driver

  Serial.begin(9600);
}

void loop() {

  int sensor = digitalRead(IR);

  if(sensor == LOW) {     // Black detected
    Serial.println("BLACK - MOTOR ON");

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 255);   // Full speed
  }
  else {
    Serial.println("WHITE - MOTOR OFF");

    analogWrite(PWMA, 0);
  }

  delay(200);
}
