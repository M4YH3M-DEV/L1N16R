/*
 * L1N16R - High-Speed 16-Sensor Line Follower Robot
 * Teensy 4.1 Firmware with PID Control and Data Logging
 * Hardware: Teensy 4.1, 16-IR sensor array, TB6612FNG motor driver
 * OPTIMIZED VERSION - Fixes SD lag, predictive recovery, 12-bit PWM
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

// Buffered logging constants
#define LOG_BUFFER_SIZE 1024
char logBufferFull[LOG_BUFFER_SIZE];
size_t logBufferIndex = 0;

// Sensor array pins (analog inputs)
const uint8_t sensorPins[NUM_SENSORS] = {
  A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15
};

// PID constants - tunable parameters
float Kp = 0.8;    // Proportional gain
float Ki = 0.0001; // Integral gain  
float Kd = 10.0;   // Derivative gain

// Motor and control variables (12-bit PWM: 0-4095)
int baseSpeed = 2950;     // Base motor speed (12-bit: ~180/255 * 4095)
int maxSpeed = 4095;      // Maximum motor speed (12-bit)
int maxCorrection = 1600; // Maximum PID correction (12-bit: ~100/255 * 4095)

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
unsigned long lastFlushTime = 0;
const unsigned long flushInterval = 1000; // Flush buffer every 1 second

// Robot states
enum RobotState {
  CALIBRATION,
  READY,
  RUNNING,
  STOPPED
};

RobotState currentState = CALIBRATION;

// Speed ramping variables
int currentSpeed = 0;
int targetSpeed = baseSpeed;

// Special case timing and predictive variables
unsigned long specialCaseTime = 0;
unsigned long lostLineTime = 0;
int lastLeftSum = 0;
int lastRightSum = 0;

void setup() {
  Serial.begin(115200);
  
  // Set PWM resolution to 12-bit for better motor control
  analogWriteResolution(12);
  
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
      
      // Periodic buffer flush during running
      if (millis() - lastFlushTime > flushInterval) {
        flushLogBuffer();
        lastFlushTime = millis();
      }
      
      // Check for stop button
      if (digitalRead(BUTTON_PIN) == LOW) {
        currentState = STOPPED;
        stopMotors();
        flushLogBuffer(); // Ensure data is saved
        digitalWrite(LED_PIN, LOW);
        Serial.println("Race stopped!");
      }
      break;
      
    case STOPPED:
      stopMotors();
      flushLogBuffer(); // Final flush when stopped
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

void flushLogBuffer() {
  if (dataFile && logBufferIndex > 0) {
    dataFile.write((uint8_t*)logBufferFull, logBufferIndex);
    dataFile.flush();
    logBufferIndex = 0;
  }
}

void calibrateSensors() {
  Serial.println("Calibrating sensors...");
  Serial.println("Move robot left and right over line");
  
  digitalWrite(LED_PIN, HIGH);
  
  // Calibrate for 10 seconds
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    
    // Simple movement during calibration (using 12-bit PWM)
    if (i < 100) {
      setMotorSpeeds(819, -819);   // Turn right (~50/255 * 4095)
    } else if (i < 200) {
      setMotorSpeeds(-819, 819);   // Turn left
    } else if (i < 300) {
      setMotorSpeeds(819, -819);   // Turn right again
    } else {
      setMotorSpeeds(-819, 819);   // Turn left again
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
    
    // Constrain PID output to 12-bit range
    if (pidValue > maxCorrection) pidValue = maxCorrection;
    if (pidValue < -maxCorrection) pidValue = -maxCorrection;
    
    // Calculate motor speeds
    targetSpeed = baseSpeed;
    
    int leftSpeed = targetSpeed + pidValue;
    int rightSpeed = targetSpeed - pidValue;
    
    // Constrain motor speeds to 12-bit range
    leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);
    
    // Smooth speed ramping
    updateSpeed();
    
    // Apply smooth ramping to calculated speeds
    float rampFactor = (float)currentSpeed / targetSpeed;
    int smoothedLeftSpeed = leftSpeed * rampFactor;
    int smoothedRightSpeed = rightSpeed * rampFactor;
    
    // Set motor speeds with 12-bit resolution
    setMotorSpeeds(smoothedLeftSpeed, smoothedRightSpeed);
    
    // Buffered data logging
    logDataBuffered();
    
    previousError = error;
    lastTime = currentTime;
  }
  
  // Handle special cases with predictive recovery
  handleSpecialCases();
}

void updateSpeed() {
  if (currentSpeed < targetSpeed) {
    currentSpeed = min(currentSpeed + 80, targetSpeed); // Faster ramp up (12-bit scale)
  } else if (currentSpeed > targetSpeed) {
    currentSpeed = max(currentSpeed - 80, targetSpeed); // Faster ramp down
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Direct 12-bit PWM control - no mapping needed
  int leftPWM = abs(leftSpeed);
  int rightPWM = abs(rightSpeed);
  
  // Constrain to 12-bit range
  leftPWM = constrain(leftPWM, 0, 4095);
  rightPWM = constrain(rightPWM, 0, 4095);
  
  // Left motor control with 12-bit precision
  if (leftSpeed >= 0) {
    digitalWrite(MOTOR_LEFT_DIR1, HIGH);
    digitalWrite(MOTOR_LEFT_DIR2, LOW);
    analogWrite(MOTOR_LEFT_PWM, leftPWM);
  } else {
    digitalWrite(MOTOR_LEFT_DIR1, LOW);
    digitalWrite(MOTOR_LEFT_DIR2, HIGH);
    analogWrite(MOTOR_LEFT_PWM, leftPWM);
  }
  
  // Right motor control with 12-bit precision
  if (rightSpeed >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
    digitalWrite(MOTOR_RIGHT_DIR2, LOW);
    analogWrite(MOTOR_RIGHT_PWM, rightPWM);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR1, LOW);
    digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
    analogWrite(MOTOR_RIGHT_PWM, rightPWM);
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
  int activeSensors = 0;
  int leftSum = 0;
  int rightSum = 0;
  
  // Calculate sensor activity and directional bias
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] > 500) activeSensors++;
    if (i < NUM_SENSORS / 2) {
      leftSum += sensorValues[i];
    } else {
      rightSum += sensorValues[i];
    }
  }
  
  // All sensors see black (intersection or end) - non-blocking
  if (activeSensors >= NUM_SENSORS - 2) {
    if (specialCaseTime == 0) {
      specialCaseTime = millis();
      stopMotors();
    } else if (millis() - specialCaseTime > 100) {
      specialCaseTime = 0;
    }
  }
  // No line detected - predictive recovery
  else if (activeSensors == 0) {
    if (lostLineTime == 0) {
      lostLineTime = millis();
      // Store last known sensor bias for prediction
      lastLeftSum = leftSum;
      lastRightSum = rightSum;
    }
    
    unsigned long lostDuration = millis() - lostLineTime;
    
    // Progressive search strategy with prediction
    if (lostDuration > 500) {
      // Aggressive search based on last sensor reading and previous error
      if (lastLeftSum > lastRightSum || previousError < 0) {
        setMotorSpeeds(-2457, 2457);  // Strong spin left (12-bit: ~-150, 150)
      } else {
        setMotorSpeeds(2457, -2457);  // Strong spin right
      }
    } else if (lostDuration > 200) {
      // Moderate search based on previous error
      if (previousError > 0) {
        setMotorSpeeds(-1310, 1310);  // Moderate spin right (12-bit: ~-80, 80)
      } else {
        setMotorSpeeds(1310, -1310);  // Moderate spin left
      }
    } else if (lostDuration > 50) {
      // Gentle search - maintain last direction briefly
      if (previousError > 0) {
        setMotorSpeeds(819, 1638);   // Gentle curve right
      } else {
        setMotorSpeeds(1638, 819);   // Gentle curve left
      }
    }
  } else {
    // Reset lost line timer when line found
    lostLineTime = 0;
    specialCaseTime = 0;
  }
}

void createLogFile() {
  dataFile = SD.open("L1N16R_log.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("Time,Position,Error,PID,LeftSpeed,RightSpeed,S0,S1,S2,S3,S4,S5,S6,S7,S8,S9,S10,S11,S12,S13,S14,S15");
    dataFile.flush();
    logging = true;
    Serial.println("Log file created: L1N16R_log.csv");
  }
}

void logDataBuffered() {
  if (!logging || (millis() - lastLogTime) < logInterval) return;
  
  // Prepare data line efficiently
  char tempBuffer[128];
  int len = snprintf(tempBuffer, sizeof(tempBuffer), "%lu,%d,%.2f,%.2f,%d,%d",
                     millis(), position, error, pidValue,
                     (int)(baseSpeed + pidValue), (int)(baseSpeed - pidValue));
  
  // Add sensor values
  for (int i = 0; i < NUM_SENSORS; i++) {
    len += snprintf(tempBuffer + len, sizeof(tempBuffer) - len, ",%d", sensorValues[i]);
  }
  
  // Add newline
  len += snprintf(tempBuffer + len, sizeof(tempBuffer) - len, "\n");
  
  // Check if buffer has space
  if (logBufferIndex + len >= LOG_BUFFER_SIZE) {
    flushLogBuffer(); // Flush when buffer is full
  }
  
  // Copy to main buffer
  memcpy(logBufferFull + logBufferIndex, tempBuffer, len);
  logBufferIndex += len;
  
  lastLogTime = millis();
}

void handleSerialInput() {
  if (Serial.available() > 0) {
    char inputBuffer[32];
    size_t len = Serial.readBytesUntil('\n', inputBuffer, sizeof(inputBuffer)-1);
    inputBuffer[len] = '\0';
    
    if (strncmp(inputBuffer, "KP=", 3) == 0) {
      Kp = atof(&inputBuffer[3]);
      Serial.print("Kp set to: "); Serial.println(Kp);
    } else if (strncmp(inputBuffer, "KI=", 3) == 0) {
      Ki = atof(&inputBuffer[3]);
      Serial.print("Ki set to: "); Serial.println(Ki);
    } else if (strncmp(inputBuffer, "KD=", 3) == 0) {
      Kd = atof(&inputBuffer[3]);
      Serial.print("Kd set to: "); Serial.println(Kd);
    } else if (strncmp(inputBuffer, "SPEED=", 6) == 0) {
      int speedInput = atoi(&inputBuffer[6]);
      baseSpeed = map(speedInput, 0, 255, 0, 4095); // Convert 8-bit input to 12-bit
      targetSpeed = baseSpeed;
      Serial.print("Base speed set to: "); Serial.print(speedInput); 
      Serial.print(" (12-bit: "); Serial.print(baseSpeed); Serial.println(")");
    } else if (strcmp(inputBuffer, "STATUS") == 0) {
      printStatus();
    } else if (strcmp(inputBuffer, "CALIBRATE") == 0) {
      currentState = CALIBRATION;
      calibrateSensors();
      currentState = READY;
    }
  }
}

void printStatus() {
  Serial.println("=== L1N16R Status ===");
  Serial.print("Kp: "); Serial.println(Kp);
  Serial.print("Ki: "); Serial.println(Ki);
  Serial.print("Kd: "); Serial.println(Kd);
  Serial.print("Base Speed: "); Serial.print(baseSpeed); Serial.println(" (12-bit)");
  Serial.print("Current Speed: "); Serial.println(currentSpeed);
  Serial.print("Position: "); Serial.println(position);
  Serial.print("Error: "); Serial.println(error);
  Serial.print("PID Value: "); Serial.println(pidValue);
  Serial.print("State: "); Serial.println(currentState);
  Serial.print("Logging: "); Serial.println(logging ? "ON" : "OFF");
  Serial.print("Buffer Usage: "); Serial.print(logBufferIndex); 
  Serial.print("/"); Serial.println(LOG_BUFFER_SIZE);
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
