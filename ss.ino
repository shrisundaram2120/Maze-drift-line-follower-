#include <EEPROM.h>

// PIN DEFINITIONS FOR ARDUINO NANO
#define MOTOR_AIN1 5
#define MOTOR_AIN2 6
#define MOTOR_BIN1 9
#define MOTOR_BIN2 10
#define MOTOR_PWMA 3
#define MOTOR_PWMB 11
#define MOTOR_STBY 4

// IR Sensor Array (8-channel) - Connected to A0-A7
int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

#define START_BUTTON 2
#define RED_LED 13

// Constants
#define NUM_SENSORS 8
#define SENSOR_THRESHOLD 500  // Adjust based on calibration
#define MAZE_SIZE 100
#define SPEED_NORMAL 150
#define SPEED_SLOW 100
#define SPEED_TURN 120
#define KP 0.08    // Proportional gain
#define KD 0.02    // Derivative gain
#define KI 0.0001  // Integral gain

// Maze directions
enum Direction { LEFT = 'L', RIGHT = 'R', STRAIGHT = 'S', BACK = 'B', UNDEFINED = 'U' };

// Global variables
int sensorValues[NUM_SENSORS];
int sensorCalMin[NUM_SENSORS];
int sensorCalMax[NUM_SENSORS];
int error = 0, lastError = 0;
float P = 0, I = 0, D = 0, PID_value = 0;
int leftMotorSpeed = 0, rightMotorSpeed = 0;
int baseSpeed = SPEED_NORMAL;

Direction path[MAZE_SIZE];
int pathIndex = 0;
bool dryRunComplete = false;
bool shortestPathMode = false;

void setup() {
  Serial.begin(9600);
  Serial.println("\nMAZEDRIFT - Arduino Nano");
  Serial.println("========================");
  
  // Initialize motor pins
  pinMode(MOTOR_AIN1, OUTPUT);
  pinMode(MOTOR_AIN2, OUTPUT);
  pinMode(MOTOR_BIN1, OUTPUT);
  pinMode(MOTOR_BIN2, OUTPUT);
  pinMode(MOTOR_PWMA, OUTPUT);
  pinMode(MOTOR_PWMB, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  
  // Initialize other pins
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(RED_LED, OUTPUT);
  
  // Initialize sensor pins
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  
  // Stop motors initially
  digitalWrite(MOTOR_STBY, LOW);
  
  // Wait for stability
  delay(1000);
  
  // Check start button for mode selection
  if (digitalRead(START_BUTTON) == LOW) {
    // Button pressed - start fresh (Dry Run)
    Serial.println("Mode: DRY RUN (Mapping)");
    blinkLED(2);  // 2 blinks for dry run
    
    dryRunComplete = false;
    shortestPathMode = false;
    pathIndex = 0;
    
    // Calibrate sensors
    calibrateSensors();
  } else {
    // Button not pressed - load saved path (Shortest Path)
    Serial.println("Mode: SHORTEST PATH");
    blinkLED(3);  // 3 blinks for shortest path
    
    loadPathFromEEPROM();
    dryRunComplete = true;
    shortestPathMode = true;
  }
  
  // Wait for button press to start
  Serial.println("Press START button to begin...");
  while(digitalRead(START_BUTTON) == HIGH) {
    delay(10);
  }
  while(digitalRead(START_BUTTON) == LOW) {
    delay(10);
  }
  
  Serial.println("GO!");
  digitalWrite(RED_LED, HIGH);
  delay(500);
  digitalWrite(RED_LED, LOW);
}

void loop() {
  if (!dryRunComplete) {
    dryRun();
  } else if (shortestPathMode) {
    shortestPathRun();
  }
}

void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(RED_LED, HIGH);
    delay(300);
    digitalWrite(RED_LED, LOW);
    delay(300);
  }
}

// ===================== DRY RUN =====================
void dryRun() {
  readSensors();
  
  // Check for end of maze
  if (isEndOfMaze()) {
    stopMotors();
    digitalWrite(RED_LED, HIGH);
    
    // Optimize and save path
    optimizePath();
    savePathToEEPROM();
    
    Serial.println("\n=== DRY RUN COMPLETE ===");
    Serial.print("Path: ");
    for (int i = 0; i < pathIndex; i++) {
      Serial.print((char)path[i]);
    }
    Serial.println();
    
    dryRunComplete = true;
    delay(3000);
    digitalWrite(RED_LED, LOW);
    return;
  }
  
  // Check for intersection
  if (isIntersection()) {
    handleIntersection();
  } else {
    // Normal line following
    calculatePID();
    motorControl(leftMotorSpeed, rightMotorSpeed);
  }
}

void handleIntersection() {
  stopMotors();
  delay(200);
  
  // Check available paths
  bool left = checkPath(LEFT);
  bool straight = checkPath(STRAIGHT);
  bool right = checkPath(RIGHT);
  
  // Decide direction using LSRB algorithm
  Direction dir = decideDirection(left, straight, right);
  
  // Store direction
  if (pathIndex < MAZE_SIZE) {
    path[pathIndex++] = dir;
    Serial.print("Stored: ");
    Serial.println((char)dir);
  }
  
  // Execute movement
  executeTurn(dir);
  
  delay(100);
}

// ===================== SHORTEST PATH RUN =====================
void shortestPathRun() {
  static int step = 0;
  
  readSensors();
  
  // Check for end
  if (isEndOfMaze()) {
    stopMotors();
    digitalWrite(RED_LED, HIGH);
    Serial.println("\n=== SHORTEST PATH COMPLETE ===");
    while(true) {
      // Blink forever
      digitalWrite(RED_LED, HIGH);
      delay(500);
      digitalWrite(RED_LED, LOW);
      delay(500);
    }
  }
  
  // Check for intersection
  if (isIntersection()) {
    stopMotors();
    delay(200);
    
    if (step < pathIndex) {
      Serial.print("Executing step ");
      Serial.print(step);
      Serial.print(": ");
      Serial.println((char)path[step]);
      
      executeTurn(path[step]);
      step++;
    } else {
      Serial.println("ERROR: Path steps exceeded!");
    }
    delay(100);
  } else {
    // Normal line following
    calculatePID();
    motorControl(leftMotorSpeed, rightMotorSpeed);
  }
}

// ===================== SENSOR FUNCTIONS =====================
void calibrateSensors() {
  Serial.println("Calibrating sensors...");
  digitalWrite(RED_LED, HIGH);
  
  // Initialize calibration arrays
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorCalMin[i] = 1023;  // Arduino ADC is 10-bit
    sensorCalMax[i] = 0;
  }
  
  // Move robot to calibrate
  digitalWrite(MOTOR_STBY, HIGH);
  
  for (int i = 0; i < 50; i++) {
    if (i < 25) {
      // Turn left
      analogWrite(MOTOR_PWMA, 80);
      analogWrite(MOTOR_PWMB, 80);
      digitalWrite(MOTOR_AIN1, LOW);
      digitalWrite(MOTOR_AIN2, HIGH);
      digitalWrite(MOTOR_BIN1, HIGH);
      digitalWrite(MOTOR_BIN2, LOW);
    } else {
      // Turn right
      analogWrite(MOTOR_PWMA, 80);
      analogWrite(MOTOR_PWMB, 80);
      digitalWrite(MOTOR_AIN1, HIGH);
      digitalWrite(MOTOR_AIN2, LOW);
      digitalWrite(MOTOR_BIN1, LOW);
      digitalWrite(MOTOR_BIN2, HIGH);
    }
    
    // Read all sensors
    for (int j = 0; j < NUM_SENSORS; j++) {
      int val = analogRead(sensorPins[j]);
      if (val < sensorCalMin[j]) sensorCalMin[j] = val;
      if (val > sensorCalMax[j]) sensorCalMax[j] = val;
    }
    delay(20);
  }
  
  stopMotors();
  digitalWrite(RED_LED, LOW);
  
  Serial.println("Calibration complete!");
  Serial.println("Sensor ranges:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("S");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(sensorCalMin[i]);
    Serial.print("-");
    Serial.println(sensorCalMax[i]);
  }
}

void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(sensorPins[i]);
    // Map to 0-1000 range
    int calibrated = map(raw, sensorCalMin[i], sensorCalMax[i], 0, 1000);
    // Binary decision (1 = black line, 0 = white surface)
    sensorValues[i] = (calibrated > SENSOR_THRESHOLD) ? 1 : 0;
  }
  
  // Debug sensor readings (optional)
  /*
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.println();
  */
}

bool isIntersection() {
  // Count sensors on line
  int count = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] == 1) count++;
  }
  return (count >= 5);  // At least 5 sensors see line
}

bool isEndOfMaze() {
  // All sensors see white
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] == 1) return false;
  }
  return true;
}

bool checkPath(Direction dir) {
  switch(dir) {
    case LEFT:
      // Quick left turn to check
      motorControl(-80, 80);
      delay(150);
      readSensors();
      stopMotors();
      // Return to original position
      motorControl(80, -80);
      delay(150);
      stopMotors();
      return (sensorValues[3] == 1 || sensorValues[4] == 1);
      
    case RIGHT:
      // Quick right turn to check
      motorControl(80, -80);
      delay(150);
      readSensors();
      stopMotors();
      // Return to original position
      motorControl(-80, 80);
      delay(150);
      stopMotors();
      return (sensorValues[3] == 1 || sensorValues[4] == 1);
      
    case STRAIGHT:
      // Move forward a bit to check
      motorControl(100, 100);
      delay(200);
      readSensors();
      stopMotors();
      // Back to original position
      motorControl(-100, -100);
      delay(200);
      stopMotors();
      return (sensorValues[3] == 1 || sensorValues[4] == 1);
      
    default:
      return false;
  }
}

// ===================== PID CONTROL =====================
void calculatePID() {
  // Calculate position (weighted average)
  long position = 0;
  long sum = 0;
  
  // Weights for each sensor (-3500 to +3500)
  int weights[NUM_SENSORS] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] == 1) {
      position += weights[i];
      sum++;
    }
  }
  
  if (sum > 0) {
    position = position / sum;
  } else {
    // No line detected - use last error
    position = lastError > 0 ? 3500 : -3500;
  }
  
  error = position;
  
  P = error;
  I += error;
  D = error - lastError;
  
  PID_value = (KP * P) + (KI * I) + (KD * D);
  lastError = error;
  
  // Calculate motor speeds
  leftMotorSpeed = baseSpeed - PID_value;
  rightMotorSpeed = baseSpeed + PID_value;
  
  // Constrain speeds
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);
}

// ===================== MOTOR CONTROL =====================
void motorControl(int leftSpeed, int rightSpeed) {
  digitalWrite(MOTOR_STBY, HIGH);
  
  // Left motor
  if (leftSpeed >= 0) {
    digitalWrite(MOTOR_AIN1, HIGH);
    digitalWrite(MOTOR_AIN2, LOW);
  } else {
    digitalWrite(MOTOR_AIN1, LOW);
    digitalWrite(MOTOR_AIN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(MOTOR_PWMA, leftSpeed);
  
  // Right motor
  if (rightSpeed >= 0) {
    digitalWrite(MOTOR_BIN1, HIGH);
    digitalWrite(MOTOR_BIN2, LOW);
  } else {
    digitalWrite(MOTOR_BIN1, LOW);
    digitalWrite(MOTOR_BIN2, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(MOTOR_PWMB, rightSpeed);
}

void stopMotors() {
  analogWrite(MOTOR_PWMA, 0);
  analogWrite(MOTOR_PWMB, 0);
  digitalWrite(MOTOR_STBY, LOW);
}

// ===================== NAVIGATION =====================
Direction decideDirection(bool left, bool straight, bool right) {
  // LSRB Algorithm (Left-Straight-Right-Back)
  if (left) return LEFT;
  if (straight) return STRAIGHT;
  if (right) return RIGHT;
  return BACK;
}

void executeTurn(Direction dir) {
  switch(dir) {
    case LEFT:
      // Turn 90 degrees left
      motorControl(-SPEED_TURN, SPEED_TURN);
      delay(400);  // Adjust this for exact 90°
      // Align to line
      while(sensorValues[3] == 0 && sensorValues[4] == 0) {
        motorControl(-80, 80);
        delay(10);
        readSensors();
      }
      break;
      
    case RIGHT:
      // Turn 90 degrees right
      motorControl(SPEED_TURN, -SPEED_TURN);
      delay(400);
      while(sensorValues[3] == 0 && sensorValues[4] == 0) {
        motorControl(80, -80);
        delay(10);
        readSensors();
      }
      break;
      
    case STRAIGHT:
      // Go straight through intersection
      motorControl(SPEED_SLOW, SPEED_SLOW);
      delay(300);
      while(sensorValues[3] == 0 && sensorValues[4] == 0) {
        motorControl(SPEED_SLOW, SPEED_SLOW);
        delay(10);
        readSensors();
      }
      break;
      
    case BACK:
      // Turn 180 degrees
      motorControl(SPEED_TURN, -SPEED_TURN);
      delay(800);  // Adjust for 180°
      break;
  }
  stopMotors();
}

// ===================== PATH OPTIMIZATION =====================
void optimizePath() {
  bool changed;
  do {
    changed = false;
    for (int i = 0; i < pathIndex - 2; i++) {
      // LBR = B (Left-Back-Right becomes Back)
      if (path[i] == LEFT && path[i+1] == BACK && path[i+2] == RIGHT) {
        path[i] = BACK;
        // Shift remaining path
        for (int j = i+1; j < pathIndex - 2; j++) {
          path[j] = path[j+2];
        }
        pathIndex -= 2;
        changed = true;
        Serial.println("Optimized: LBR -> B");
        break;
      }
      // LBS = R (Left-Back-Straight becomes Right)
      else if (path[i] == LEFT && path[i+1] == BACK && path[i+2] == STRAIGHT) {
        path[i] = RIGHT;
        for (int j = i+1; j < pathIndex - 2; j++) {
          path[j] = path[j+2];
        }
        pathIndex -= 2;
        changed = true;
        Serial.println("Optimized: LBS -> R");
        break;
      }
      // SBL = R (Straight-Back-Left becomes Right)
      else if (path[i] == STRAIGHT && path[i+1] == BACK && path[i+2] == LEFT) {
        path[i] = RIGHT;
        for (int j = i+1; j < pathIndex - 2; j++) {
          path[j] = path[j+2];
        }
        pathIndex -= 2;
        changed = true;
        Serial.println("Optimized: SBL -> R");
        break;
      }
    }
  } while (changed);
  
  Serial.print("Optimized path (");
  Serial.print(pathIndex);
  Serial.print(" steps): ");
  for (int i = 0; i < pathIndex; i++) {
    Serial.print((char)path[i]);
  }
  Serial.println();
}

// ===================== EEPROM FUNCTIONS (Arduino Nano) =====================
void savePathToEEPROM() {
  // Save path length at address 0
  EEPROM.write(0, pathIndex);
  
  // Save each direction
  for (int i = 0; i < pathIndex; i++) {
    EEPROM.write(i + 1, (byte)path[i]);
  }
  
  Serial.println("Path saved to EEPROM");
}

void loadPathFromEEPROM() {
  pathIndex = EEPROM.read(0);
  
  if (pathIndex > MAZE_SIZE || pathIndex == 255) {
    pathIndex = 0;
    Serial.println("No valid path in EEPROM, starting fresh");
    return;
  }
  
  for (int i = 0; i < pathIndex; i++) {
    path[i] = (Direction)EEPROM.read(i + 1);
  }
  
  Serial.print("Loaded path from EEPROM (");
  Serial.print(pathIndex);
  Serial.print(" steps): ");
  for (int i = 0; i < pathIndex; i++) {
    Serial.print((char)path[i]);
  }
  Serial.println();
}

// ===================== TEST FUNCTIONS =====================
void testHardware() {
  Serial.println("\n=== HARDWARE TEST ===");
  
  // Test LED
  Serial.println("Testing LED...");
  for (int i = 0; i < 5; i++) {
    digitalWrite(RED_LED, HIGH);
    delay(200);
    digitalWrite(RED_LED, LOW);
    delay(200);
  }
  
  // Test button
  Serial.println("Press START button...");
  int timeout = 0;
  while(digitalRead(START_BUTTON) == HIGH && timeout < 200) {
    delay(10);
    timeout++;
  }
  if (digitalRead(START_BUTTON) == LOW) {
    Serial.println("Button pressed!");
    delay(500);
  } else {
    Serial.println("Button not pressed (test skipped)");
  }
  
  // Test sensors
  Serial.println("Testing sensors... Place over black/white:");
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < NUM_SENSORS; j++) {
      Serial.print(analogRead(sensorPins[j]));
      Serial.print("\t");
    }
    Serial.println();
    delay(500);
  }
  
  // Test motors
  Serial.println("Testing motors...");
  motorControl(100, 100);
  delay(1000);
  stopMotors();
  delay(500);
  
  motorControl(-100, 100);
  delay(500);
  stopMotors();
  delay(500);
  
  motorControl(100, -100);
  delay(500);
  stopMotors();
  
  Serial.println("Hardware test complete!");
}