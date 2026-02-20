// ===== MOTOR DRIVER PINS =====
#define AIN1 5
#define AIN2 6
#define PWMA 3

#define BIN1 9
#define BIN2 10
#define PWMB 11

#define STBY 4

// ===== ANALOG SENSOR PINS =====
int sensorPins[8] = {A0,A1,A2,A3,A4,A5,A6,A7};
int sensorValue[8];

// ===== PID =====
float Kp = 0.07;
float Ki = 0.0001;
float Kd = 0.03;

float error = 0;
float lastError = 0;
float integral = 0;

int baseSpeed = 160;

void setup()
{
  Serial.begin(9600);

  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(STBY,OUTPUT);

  digitalWrite(STBY,HIGH);
}

void loop()
{
  int weights[8] = {-3500,-2500,-1500,-500,500,1500,2500,3500};
  long position = 0;
  int count = 0;

  for(int i=0;i<8;i++)
  {
    sensorValue[i] = analogRead(sensorPins[i]);

    if(sensorValue[i] > 500)  // adjust if needed
    {
      position += weights[i];
      count++;
    }
  }

  if(count > 0)
  {
    error = position / count;
    applyPID();
  }
  else
  {
    stopMotors();
  }
}

void applyPID()
{
  float P = error;
  integral += error;
  float D = error - lastError;

  float correction = (Kp*P) + (Ki*integral) + (Kd*D);
  lastError = error;

  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed  = constrain(leftSpeed,0,255);
  rightSpeed = constrain(rightSpeed,0,255);

  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,LOW);

  analogWrite(PWMA,leftSpeed);
  analogWrite(PWMB,rightSpeed);
}

void stopMotors()
{
  analogWrite(PWMA,0);
  analogWrite(PWMB,0);
}