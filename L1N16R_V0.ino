/*
 * L1N16R - High-Speed 16-Sensor Line Follower Robot
 * Teensy 4.1 Firmware with PID Control and Data Logging
 * Hardware: Teensy 4.1, 16-IR sensor array, TB6612FNG motor driver
 */

#include <QTRSensors.h>
#include <SD.h>
#include <SPI.h>

// Hardware pin definitions for custom PCB
#define NUM_SENSORS 16
#define MOTOR_LEFT_PWM 5
#define MOTOR_RIGHT_PWM 6
#define MOTOR_LEFT_DIR1 7
#define MOTOR_LEFT_DIR2 8
#define MOTOR_RIGHT_DIR1 9
#define MOTOR_RIGHT_DIR2 10
#define BUTTON_PIN 12
#define LED_PIN 13
#define SD_CS_PIN 254  // Teensy 4.1 built-in SD

// Sensor array pins (analog inputs)
const uint8_t sensorPins[NUM_SENSORS] = {
  A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15
};

// PID constants - tunable parameters
float Kp = 0.8;    // Proportional gain
float Ki = 0.0001; // Integral gain  
float Kd = 10.0;   // Derivative gain

// Motor and control variables
int baseSpeed = 180;    // Base motor speed (0-255)
int maxSpeed = 255;     // Maximum motor speed
int maxCorrection = 100; // Maximum PID correction

// PID control variables
float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;
float pidValue = 0;

// Sensor and timing variables
QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];
uint16_t position;
unsigned long lastTime = 0;
unsigned long currentTime = 0;

// Data logging variables
File dataFile;
bool logging = false;
unsigned long logInterval = 50; // Log every 50ms
unsigned long lastLogTime = 0;

// Robot states
enum RobotState {
  CALIBRATION,
  READY,
  RUNNING,
  STOPPED
};

RobotState currentState = CALIBRATION;

void setup() {
  Serial.begin(115200);
  
  // Initialize hardware pins
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR2, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR2, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize sensor array
  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, NUM_SENSORS);
  
  // Initialize SD card for data logging
  if (SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialized successfully");
    createLogFile();
  } else {
    Serial.println("SD card initialization failed");
  }
  
  Serial.println("L1N16R Initialization Complete");
  Serial.println("Press button to start calibration");
  
  // Wait for button press to start calibration
  waitForButtonPress();
  calibrateSensors();
  
  currentState = READY;
  Serial.println("Ready to race! Press button to start.");
}

void loop() {
  switch (currentState) {
    case READY:
      if (digitalRead(BUTTON_PIN) == LOW) {
        delay(100); // Debounce
        if (digitalRead(BUTTON_PIN) == LOW) {
          currentState = RUNNING;
          Serial.println("Race started!");
          digitalWrite(LED_PIN, HIGH);
        }
      }
      break;
      
    case RUNNING:
      followLine();
      
      // Check for stop button
      if (digitalRead(BUTTON_PIN) == LOW) {
        currentState = STOPPED;
        stopMotors();
        digitalWrite(LED_PIN, LOW);
        Serial.println("Race stopped!");
      }
      break;
      
    case STOPPED:
      stopMotors();
      if (digitalRead(BUTTON_PIN) == LOW) {
        delay(100);
        if (digitalRead(BUTTON_PIN) == LOW) {
          currentState = READY;
          Serial.println("Ready to race again!");
        }
      }
      break;
  }
  
  // Handle serial commands for PID tuning
  handleSerialInput();
}

void calibrateSensors() {
  Serial.println("Calibrating sensors...");
  Serial.println("Move robot left and right over line");
  
  digitalWrite(LED_PIN, HIGH);
  
  // Calibrate for 10 seconds
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    
    // Simple movement during calibration
    if (i < 100) {
      setMotorSpeeds(50, -50);   // Turn right
    } else if (i < 200) {
      setMotorSpeeds(-50, 50);   // Turn left
    } else if (i < 300) {
      setMotorSpeeds(50, -50);   // Turn right again
    } else {
      setMotorSpeeds(-50, 50);   // Turn left again
    }
    
    delay(20);
  }
  
  stopMotors();
  digitalWrite(LED_PIN, LOW);
  
  // Print calibration results
  Serial.println("Calibration complete!");
  Serial.print("Minimums: ");
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  Serial.print("Maximums: ");
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
}

void followLine() {
  // Read sensor array and calculate line position
  position = qtr.readLineBlack(sensorValues);
  
  // Calculate error (0-15000 range, center = 7500)
  error = position - 7500;
  
  // Calculate PID components
  currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
  
  if (deltaTime > 0) {
    integral += error * deltaTime;
    derivative = (error - previousError) / deltaTime;
    
    // Anti-windup for integral
    if (integral > 1000) integral = 1000;
    if (integral < -1000) integral = -1000;
    
    // Calculate PID output
    pidValue = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    // Constrain PID output
    if (pidValue > maxCorrection) pidValue = maxCorrection;
    if (pidValue < -maxCorrection) pidValue = -maxCorrection;
    
    // Calculate motor speeds
    int leftSpeed = baseSpeed + pidValue;
    int rightSpeed = baseSpeed - pidValue;
    
    // Constrain motor speeds
    leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);
    
    // Set motor speeds
    setMotorSpeeds(leftSpeed, rightSpeed);
    
    // Data logging
    logData();
    
    previousError = error;
    lastTime = currentTime;
  }
  
  // Handle special cases
  handleSpecialCases();
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Left motor control
  if (leftSpeed >= 0) {
    digitalWrite(MOTOR_LEFT_DIR1, HIGH);
    digitalWrite(MOTOR_LEFT_DIR2, LOW);
    analogWrite(MOTOR_LEFT_PWM, leftSpeed);
  } else {
    digitalWrite(MOTOR_LEFT_DIR1, LOW);
    digitalWrite(MOTOR_LEFT_DIR2, HIGH);
    analogWrite(MOTOR_LEFT_PWM, -leftSpeed);
  }
  
  // Right motor control
  if (rightSpeed >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
    digitalWrite(MOTOR_RIGHT_DIR2, LOW);
    analogWrite(MOTOR_RIGHT_PWM, rightSpeed);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR1, LOW);
    digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
    analogWrite(MOTOR_RIGHT_PWM, -rightSpeed);
  }
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  digitalWrite(MOTOR_RIGHT_DIR1, LOW);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
}

void handleSpecialCases() {
  // Count active sensors
  int activeSensors = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] > 500) activeSensors++; // Adjust threshold as needed
  }
  
  // All sensors see black (intersection or end)
  if (activeSensors >= NUM_SENSORS - 2) {
    // Stop or perform specific action
    stopMotors();
    delay(100);
  }
  
  // No line detected
  else if (activeSensors == 0) {
    // Last known direction recovery
    if (previousError > 0) {
      setMotorSpeeds(0, 100); // Turn right to find line
    } else {
      setMotorSpeeds(100, 0); // Turn left to find line
    }
  }
}

void createLogFile() {
  // Create timestamped log file
  String filename = "L1N16R_" + String(millis()) + ".csv";
  dataFile = SD.open(filename.c_str(), FILE_WRITE);
  
  if (dataFile) {
    // Write CSV header
    dataFile.println("Time,Position,Error,PID,LeftSpeed,RightSpeed,S0,S1,S2,S3,S4,S5,S6,S7,S8,S9,S10,S11,S12,S13,S14,S15");
    dataFile.close();
    Serial.println("Log file created: " + filename);
    logging = true;
  }
}

void logData() {
  if (!logging || (millis() - lastLogTime) < logInterval) return;
  
  String filename = "L1N16R_" + String(millis()) + ".csv";
  dataFile = SD.open(filename.c_str(), FILE_WRITE);
  
  if (dataFile) {
    dataFile.print(millis());
    dataFile.print(",");
    dataFile.print(position);
    dataFile.print(",");
    dataFile.print(error);
    dataFile.print(",");
    dataFile.print(pidValue);
    dataFile.print(",");
    dataFile.print(baseSpeed + pidValue);
    dataFile.print(",");
    dataFile.print(baseSpeed - pidValue);
    
    // Log all sensor values
    for (int i = 0; i < NUM_SENSORS; i++) {
      dataFile.print(",");
      dataFile.print(sensorValues[i]);
    }
    dataFile.println();
    dataFile.close();
  }
  
  lastLogTime = millis();
}

void handleSerialInput() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("KP=")) {
      Kp = command.substring(3).toFloat();
      Serial.println("Kp set to: " + String(Kp));
    }
    else if (command.startsWith("KI=")) {
      Ki = command.substring(3).toFloat();
      Serial.println("Ki set to: " + String(Ki));
    }
    else if (command.startsWith("KD=")) {
      Kd = command.substring(3).toFloat();
      Serial.println("Kd set to: " + String(Kd));
    }
    else if (command.startsWith("SPEED=")) {
      baseSpeed = command.substring(6).toInt();
      Serial.println("Base speed set to: " + String(baseSpeed));
    }
    else if (command == "STATUS") {
      printStatus();
    }
    else if (command == "CALIBRATE") {
      currentState = CALIBRATION;
      calibrateSensors();
      currentState = READY;
    }
  }
}

void printStatus() {
  Serial.println("=== L1N16R Status ===");
  Serial.println("Kp: " + String(Kp));
  Serial.println("Ki: " + String(Ki));
  Serial.println("Kd: " + String(Kd));
  Serial.println("Base Speed: " + String(baseSpeed));
  Serial.println("Position: " + String(position));
  Serial.println("Error: " + String(error));
  Serial.println("PID Value: " + String(pidValue));
  Serial.println("State: " + String(currentState));
  Serial.println("Logging: " + String(logging ? "ON" : "OFF"));
  Serial.println("====================");
}

void waitForButtonPress() {
  while (digitalRead(BUTTON_PIN) == HIGH) {
    delay(10);
  }
  delay(100); // Debounce
  while (digitalRead(BUTTON_PIN) == LOW) {
    delay(10);
  }
}
