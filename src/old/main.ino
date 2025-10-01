/*
    Copyright (C) 2025  Alexis Martinez

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

/*
 * WRO Future Engineers Robot Controller
 * Arduino Mega 2560 with TCS3200 RGB sensor and advanced navigation
 * 
 * Hardware Configuration:
 * - TCS3200 RGB Sensor: S0=23, S1=25, S2=27, S3=29, OUT=31
 * - MPU6050 IMU: SDA=20, SCL=21
 * - Left Distance Sensor: TRIG=13, ECHO=A11
 * - Right Distance Sensor: TRIG=12, ECHO=A12
 * - Front Distance Sensor: TRIG=4, ECHO=3
 * - Drive Motor: PWM=11
 * - Wheel Encoder: A=22, B=24
 * - Steering Servo: PWM=5
 * - NeoPixel Status: PIN=8, COUNT=2
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// ===== HARDWARE PIN ASSIGNMENTS =====
// TCS3200 RGB Color Sensor
#define RGB_S0_PIN    23
#define RGB_S1_PIN    25
#define RGB_S2_PIN    27
#define RGB_S3_PIN    29
#define RGB_OUT_PIN   31

// MPU6050 Inertial Measurement Unit
#define IMU_SDA_PIN   20
#define IMU_SCL_PIN   21

// HC-SR04 Distance Sensors
#define DIST_LEFT_TRIG  13
#define DIST_LEFT_ECHO  A11
#define DIST_RIGHT_TRIG 12
#define DIST_RIGHT_ECHO A12
#define DIST_FRONT_TRIG 4
#define DIST_FRONT_ECHO 3

// Actuators and Feedback
#define DRIVE_MOTOR_PIN 11
#define STEERING_SERVO_PIN 5
#define ENCODER_PHASE_A 22
#define ENCODER_PHASE_B 24

// Status and Control
#define STATUS_LED_PIN  8
#define STATUS_LED_COUNT 2
#define START_BUTTON_PIN A0
#define DEBUG_LED_PIN   2

// ===== DRIVE MOTOR MANAGEMENT =====
class DriveSystem {
private:
  int motorPin;
  int currentPower;
  
public:
  DriveSystem(int pin) : motorPin(pin), currentPower(0) {}
  
  void initialize() {
    pinMode(motorPin, OUTPUT);
    analogWrite(motorPin, 0);
  }
  
  void setPower(int power) {
    power = constrain(power, 0, 255);
    currentPower = power;
    analogWrite(motorPin, power);
  }
  
  void halt() {
    setPower(0);
  }
  
  int getCurrentPower() { return currentPower; }
};

// ===== STEERING SERVO MANAGEMENT =====
class SteeringSystem {
private:
  Servo steeringServo;
  int currentPosition;
  const int CENTER_POSITION = 90;
  const int LEFT_BOUNDARY = 50;   
  const int RIGHT_BOUNDARY = 130; 
  
public:
  void initialize() {
    steeringServo.attach(STEERING_SERVO_PIN);
    currentPosition = CENTER_POSITION;
    steeringServo.write(CENTER_POSITION);
    delay(500); // Allow servo to reach position
    
    Serial.print("Steering initialized at: ");
    Serial.println(CENTER_POSITION);
  }
  
  void setPosition(int position) {
    position = constrain(position, LEFT_BOUNDARY, RIGHT_BOUNDARY);
    currentPosition = position;
    steeringServo.write(position);
  }
  
  void centerSteering() {
    setPosition(CENTER_POSITION);
  }
  
  void steerLeft(int amount) {
    setPosition(CENTER_POSITION - amount);
  }
  
  void steerRight(int amount) {
    setPosition(CENTER_POSITION + amount);
  }
  
  int getPosition() { return currentPosition; }
};

// ===== VISUAL STATUS INDICATOR =====
class StatusDisplay {
private:
  Adafruit_NeoPixel ledStrip;
  
public:
  StatusDisplay() : ledStrip(STATUS_LED_COUNT, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800) {}
  
  void initialize() {
    ledStrip.begin();
    ledStrip.show(); 
    Serial.println("Status display initialized");
  }
  
  void setAllLEDs(uint8_t r, uint8_t g, uint8_t b) {
    for(int i = 0; i < STATUS_LED_COUNT; i++) {
      ledStrip.setPixelColor(i, ledStrip.Color(r, g, b));
    }
    ledStrip.show();
  }
  
  void setSingleLED(int index, uint8_t r, uint8_t g, uint8_t b) {
    if(index >= 0 && index < STATUS_LED_COUNT) {
      ledStrip.setPixelColor(index, ledStrip.Color(r, g, b));
      ledStrip.show();
    }
  }
  
  void clearDisplay() { setAllLEDs(0, 0, 0); }
  void showRed() { setAllLEDs(255, 0, 0); }
  void showGreen() { setAllLEDs(0, 255, 0); }
  void showBlue() { setAllLEDs(0, 0, 255); }
  void showOrange() { setAllLEDs(255, 165, 0); }
  void showWhite() { setAllLEDs(255, 255, 255); }
  
  void adjustBrightness(uint8_t level) {
    ledStrip.setBrightness(level);
    ledStrip.show();
  }
  
  void flashPattern(uint8_t r, uint8_t g, uint8_t b, int duration = 500) {
    setAllLEDs(r, g, b);
    delay(duration);
    clearDisplay();
    delay(duration);
  }
};

// ===== RGB COLOR DETECTION SYSTEM =====
class ColorDetectionSystem {
private:
  // Detection thresholds for TCS3200 sensor
  struct ColorThresholds {
    int redMin, redMax;
    int greenMin, greenMax;  
    int blueMin, blueMax;
  };
  
  ColorThresholds blueBlockThresh = {50, 100, 50, 100, 150, 255};
  ColorThresholds orangeBlockThresh = {150, 255, 100, 180, 50, 120};
  
public:
  enum DetectedColor { NO_COLOR, BLUE_BLOCK, ORANGE_BLOCK };
  
  void initialize() {
    pinMode(RGB_S0_PIN, OUTPUT);
    pinMode(RGB_S1_PIN, OUTPUT);
    pinMode(RGB_S2_PIN, OUTPUT);
    pinMode(RGB_S3_PIN, OUTPUT);
    pinMode(RGB_OUT_PIN, INPUT);
    
    // Configure frequency scaling to 20%
    digitalWrite(RGB_S0_PIN, HIGH);
    digitalWrite(RGB_S1_PIN, LOW);
  }
  
  int measureChannel(char channel) {
    // Configure photodiode filters
    switch(channel) {
      case 'R':
        digitalWrite(RGB_S2_PIN, LOW);
        digitalWrite(RGB_S3_PIN, LOW);
        break;
      case 'G':
        digitalWrite(RGB_S2_PIN, HIGH);
        digitalWrite(RGB_S3_PIN, HIGH);
        break;
      case 'B':
        digitalWrite(RGB_S2_PIN, LOW);
        digitalWrite(RGB_S3_PIN, HIGH);
        break;
    }
    
    delayMicroseconds(100);
    return pulseIn(RGB_OUT_PIN, LOW, 50000);
  }
  
  DetectedColor analyzeColor() {
    int redValue = measureChannel('R');
    int greenValue = measureChannel('G');
    int blueValue = measureChannel('B');
    
    // TCS3200: lower frequency indicates higher color intensity
    
    if (blueValue < blueBlockThresh.blueMax && 
        redValue > blueBlockThresh.redMin && 
        greenValue > blueBlockThresh.greenMin) {
      return BLUE_BLOCK;
    }
    
    if (redValue < orangeBlockThresh.redMax && 
        greenValue < orangeBlockThresh.greenMax && 
        blueValue > orangeBlockThresh.blueMin) {
      return ORANGE_BLOCK;
    }
    
    return NO_COLOR;
  }
};

// ===== DISTANCE MEASUREMENT ARRAY =====
class ProximitySensorArray {
private:
  int leftTrigPin, leftEchoPin;
  int rightTrigPin, rightEchoPin;
  int frontTrigPin, frontEchoPin;
  
  int performSingleMeasurement(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration == 0) return 0;
    
    return duration * 0.034 / 2; // Convert to centimeters
  }
  
  int getStableMeasurement(int trigPin, int echoPin) {
    const int SAMPLE_COUNT = 5;
    const int MAX_DISTANCE = 100;
    const int MIN_DISTANCE = 2;
    const int VARIANCE_LIMIT = 10;
    
    int readings[SAMPLE_COUNT];
    int validReadings = 0;
    int sum = 0;

    for (int i = 0; i < SAMPLE_COUNT; i++) {
      int distance = performSingleMeasurement(trigPin, echoPin);
      delay(3); // Prevent interference
      
      if (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE) {
        readings[validReadings++] = distance;  
        sum += distance;
      }
    }

    if (validReadings == 0) return 0;

    // Calculate average
    int average = sum / validReadings;

    // Filter outliers
    sum = 0;
    int filteredCount = 0;
    for (int i = 0; i < validReadings; i++) {
      if (abs(readings[i] - average) <= VARIANCE_LIMIT) {
        sum += readings[i];
        filteredCount++;
      }
    }

    if (filteredCount == 0) return 0;
    return sum / filteredCount;
  }
  
public:
  ProximitySensorArray(int lTrig, int lEcho, int rTrig, int rEcho, int fTrig, int fEcho) :
    leftTrigPin(lTrig), leftEchoPin(lEcho),
    rightTrigPin(rTrig), rightEchoPin(rEcho),
    frontTrigPin(fTrig), frontEchoPin(fEcho) {}
  
  void initialize() {
    pinMode(leftTrigPin, OUTPUT);
    pinMode(leftEchoPin, INPUT);
    pinMode(rightTrigPin, OUTPUT);
    pinMode(rightEchoPin, INPUT);
    pinMode(frontTrigPin, OUTPUT);
    pinMode(frontEchoPin, INPUT);
  }
  
  int getLeftDistance() {
    return getStableMeasurement(leftTrigPin, leftEchoPin);
  }
  
  int getRightDistance() {
    return getStableMeasurement(rightTrigPin, rightEchoPin);
  }
  
  int getFrontDistance() {
    return getStableMeasurement(frontTrigPin, frontEchoPin);
  }
  
  int getCenterDistance() {
    return getStableMeasurement(frontTrigPin, frontEchoPin);
  }
  
  // Select appropriate sensor based on navigation direction
  int getDirectionalDistance(int navDirection) {
    if (navDirection < 0) {
      // Left navigation, monitor right wall
      return getRightDistance();
    } else {
      // Right navigation, monitor left wall
      return getLeftDistance();
    }
  }
};

// ===== INERTIAL NAVIGATION SYSTEM =====
class InertialNavigator {
private:
  Adafruit_MPU6050 imuSensor;
  float currentHeading;
  float targetHeading;
  float calibrationOffset;
  float lastError;
  
  // Navigation control constants
  const float PROPORTIONAL_GAIN = 1.8;
  const float DERIVATIVE_GAIN = 1.2;
  
public:
  InertialNavigator() : currentHeading(0), targetHeading(0), calibrationOffset(0), lastError(0) {}
  
  bool initialize() {
    Wire.begin(); // Arduino Mega hardware I2C pins 20/21
    
    if (!imuSensor.begin()) {
      return false;
    }
    
    imuSensor.setAccelerometerRange(MPU6050_RANGE_8_G);
    imuSensor.setGyroRange(MPU6050_RANGE_500_DEG);
    imuSensor.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    performCalibration();
    return true;
  }
  
  void performCalibration() {
    const int CALIBRATION_SAMPLES = 250;
    float offsetSum = 0;
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
      sensors_event_t accel, gyro, temp;
      imuSensor.getEvent(&accel, &gyro, &temp);
      offsetSum += gyro.gyro.z;
      delay(5);
    }
    
    calibrationOffset = offsetSum / CALIBRATION_SAMPLES;
  }
  
  void updateHeading(float deltaTime) {
    sensors_event_t accel, gyro, temp;
    imuSensor.getEvent(&accel, &gyro, &temp);
    
    float gyroRate = (gyro.gyro.z - calibrationOffset) * 57.2958; // Convert to deg/sec
    
    if (abs(gyroRate) > 0.1) { // Filter noise
      currentHeading += gyroRate * deltaTime;
    }
  }
  
  int calculateSteeringCorrection() {
    float headingError = targetHeading - currentHeading;
    float errorDerivative = headingError - lastError;
    
    int correction = -(int)(PROPORTIONAL_GAIN * headingError + DERIVATIVE_GAIN * errorDerivative);
    lastError = headingError;
    
    return correction;
  }
  
  void setTargetHeading(float target) { targetHeading = target; }
  float getCurrentHeading() { return currentHeading; }
  float getTargetHeading() { return targetHeading; }
  float getHeadingError() { return abs(targetHeading - currentHeading); }
};

// ===== WHEEL ROTATION TRACKER =====
class WheelEncoder {
private:
  volatile long rotationCount;
  static WheelEncoder* activeInstance;
  
  static void encoderInterrupt() {
    if (activeInstance) {
      activeInstance->rotationCount++;
    }
  }
  
public:
  WheelEncoder() : rotationCount(0) {
    activeInstance = this;
  }
  
  void initialize() {
    pinMode(ENCODER_PHASE_A, INPUT_PULLUP);
    pinMode(ENCODER_PHASE_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PHASE_A), encoderInterrupt, RISING);
  }
  
  long getRotationCount() { return rotationCount; }
  void resetCount() { rotationCount = 0; }
  
  float calculateDistance() {
    const float WHEEL_CIRCUMFERENCE = 19.48; // cm
    const float PULSES_PER_ROTATION = 341.2; // Encoder resolution
    return (rotationCount * WHEEL_CIRCUMFERENCE) / PULSES_PER_ROTATION;
  }
};

// Static instance definition
WheelEncoder* WheelEncoder::activeInstance = nullptr;

// ===== AUTONOMOUS NAVIGATION CONTROLLER =====
class AutonomousNavigator {
private:
  DriveSystem driveMotor;
  SteeringSystem steeringServo;
  StatusDisplay statusLEDs;
  ColorDetectionSystem colorSensor;
  ProximitySensorArray distanceSensors;
  InertialNavigator headingControl;
  WheelEncoder motionTracker;
  
  // Navigation state management
  enum NavigationMode { CRUISING, MANEUVERING, ADJUSTING };
  NavigationMode currentMode;
  
  int navigationDirection; // -1 = left course, 1 = right course, 0 = undetermined
  int completedManeuvers;
  bool maneuverInProgress;
  unsigned long lastUpdateTime;
  unsigned long lastAdjustmentTime;
  
  // Speed parameters
  int cruisingSpeed = 70;
  int maneuverSpeed = 150;
  
public:
  AutonomousNavigator() : 
    driveMotor(DRIVE_MOTOR_PIN),
    distanceSensors(DIST_LEFT_TRIG, DIST_LEFT_ECHO, DIST_RIGHT_TRIG, DIST_RIGHT_ECHO, DIST_FRONT_TRIG, DIST_FRONT_ECHO),
    currentMode(CRUISING),
    navigationDirection(0),
    completedManeuvers(0),
    maneuverInProgress(false),
    lastUpdateTime(0),
    lastAdjustmentTime(0) {}
  
  bool initializeAllSystems() {
    Serial.begin(115200);
    
    driveMotor.initialize();
    steeringServo.initialize();
    statusLEDs.initialize();
    colorSensor.initialize();
    distanceSensors.initialize();
    motionTracker.initialize();
    
    if (!headingControl.initialize()) {
      Serial.println("Failed to initialize heading control!");
      return false;
    }
    
    pinMode(DEBUG_LED_PIN, OUTPUT);
    pinMode(START_BUTTON_PIN, INPUT);
    
    Serial.println("All systems initialized successfully");
    return true;
  }
  
  void waitForStartSignal() {
    Serial.println("Waiting for start signal...");
    digitalWrite(DEBUG_LED_PIN, HIGH);
    statusLEDs.showRed(); // Red indicates waiting
    
    while (!digitalRead(START_BUTTON_PIN)) {
      delay(10);
    }
    
    digitalWrite(DEBUG_LED_PIN, LOW);
    statusLEDs.showGreen(); // Green indicates ready
    delay(1000);
    statusLEDs.clearDisplay();
    Serial.println("Navigation commenced...");
  }
  
  void executeNavigationCycle() {
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime < 30) return; // 33Hz update rate
    
    float timeStep = (currentTime - lastUpdateTime) / 1000.0;
    lastUpdateTime = currentTime;
    
    headingControl.updateHeading(timeStep);
    
    switch (currentMode) {
      case CRUISING:
        performCruisingBehavior();
        break;
      case MANEUVERING:
        performManeuveringBehavior();
        break;
      case ADJUSTING:
        performAdjustingBehavior();
        break;
    }
    
    checkForMissionCompletion();
  }
  
private:
  void performCruisingBehavior() {
    driveMotor.setPower(cruisingSpeed);
    
    // Apply heading correction
    int steeringAdjustment = headingControl.calculateSteeringCorrection();
    steeringServo.setPosition(90 + steeringAdjustment);
    
    // Establish navigation direction on first color detection
    if (navigationDirection == 0) {
      ColorDetectionSystem::DetectedColor blockColor = colorSensor.analyzeColor();
      if (blockColor != ColorDetectionSystem::NO_COLOR && !maneuverInProgress) {
        determineNavigationDirection(blockColor);
        return;
      }
    }
    
    // Monitor for obstacle requiring maneuver
    checkForObstacleManeuver();
    
    // Perform path centering corrections
    performPathCentering();
  }
  
  void determineNavigationDirection(ColorDetectionSystem::DetectedColor color) {
    if (completedManeuvers == 0) {
      // Establish permanent navigation direction
      navigationDirection = (color == ColorDetectionSystem::BLUE_BLOCK) ? -1 : 1;
      Serial.print("Navigation direction established: ");
      Serial.println(navigationDirection == -1 ? "LEFT course (Blue)" : "RIGHT course (Orange)");
      Serial.println("Switching to obstacle-based navigation");
    }
    
    // After direction establishment, ignore color detection
  }
  
  void performManeuveringBehavior() {
    // Engine control during maneuvers
    if (completedManeuvers < 12) {
      driveMotor.setPower(maneuverSpeed);
    }
    
    int steeringAdjustment = headingControl.calculateSteeringCorrection();
    steeringServo.setPosition(90 + steeringAdjustment);
    
    if (headingControl.getHeadingError() < 3.0) {
      // Maneuver completed
      digitalWrite(DEBUG_LED_PIN, LOW);
      statusLEDs.clearDisplay();
      maneuverInProgress = false;
      currentMode = CRUISING;
      
      // Resume normal speed only if not final maneuver
      if (completedManeuvers < 12) {
        driveMotor.setPower(cruisingSpeed);
      }
      
      Serial.println("Maneuver completed");
    }
  }
  
  // Detect obstacles requiring navigation maneuvers
  void checkForObstacleManeuver() {
    if (navigationDirection == 0 || maneuverInProgress) return;
    
    int frontObstacleDistance = distanceSensors.getFrontDistance();
    
    // Initiate maneuver when obstacle detected
    if (frontObstacleDistance > 0 && frontObstacleDistance < 50) {
      completedManeuvers++;
      digitalWrite(DEBUG_LED_PIN, HIGH);
      
      // Visual feedback based on maneuver direction
      if (navigationDirection == -1) {
        statusLEDs.showBlue(); // Blue for left maneuvers
      } else {
        statusLEDs.showOrange(); // Orange for right maneuvers
      }
      
      // Calculate new heading target
      float newHeading = headingControl.getTargetHeading() + (90.0 * -navigationDirection);
      headingControl.setTargetHeading(newHeading);
      
      // Special handling for final maneuver
      if (completedManeuvers >= 13) {
        driveMotor.halt(); // Engine off for final maneuver
        Serial.println("FINAL MANEUVER - Engine shutdown");
      } else {
        driveMotor.setPower(maneuverSpeed);
      }
      
      maneuverInProgress = true;
      currentMode = MANEUVERING;
      
      Serial.print("Obstacle at ");
      Serial.print(frontObstacleDistance);
      Serial.print("cm - Maneuver ");
      Serial.print(completedManeuvers);
      Serial.print(" initiated (");
      Serial.print(navigationDirection == -1 ? "LEFT" : "RIGHT");
      Serial.println(")");
    }
  }
  
  // Maintain centered path between boundaries
  void performPathCentering() {
    if (maneuverInProgress || navigationDirection == 0) return;
    if (millis() - lastAdjustmentTime < 800) return;
    
    // Read bilateral distance sensors
    int leftBoundary = distanceSensors.getLeftDistance();
    int rightBoundary = distanceSensors.getRightDistance();
    
    if (leftBoundary == 0 || rightBoundary == 0) return;
    
    const int CENTERING_MAGNITUDE = 6;
    const int CENTERING_TOLERANCE = 8;
    
    int boundaryDifference = leftBoundary - rightBoundary;
    
    if (abs(boundaryDifference) > CENTERING_TOLERANCE) {
      float adjustedHeading;
      if (boundaryDifference > 0) {
        // Closer to left boundary, adjust rightward
        adjustedHeading = headingControl.getTargetHeading() + CENTERING_MAGNITUDE;
        Serial.println("Path centering: adjusting right");
      } else {
        // Closer to right boundary, adjust leftward
        adjustedHeading = headingControl.getTargetHeading() - CENTERING_MAGNITUDE;
        Serial.println("Path centering: adjusting left");
      }
      
      headingControl.setTargetHeading(adjustedHeading);
      lastAdjustmentTime = millis();
    }
  }
  
  void performAdjustingBehavior() {
    // Reserved for future correction algorithms
    currentMode = CRUISING;
  }
  
  void checkForMissionCompletion() {
    if (completedManeuvers >= 12) {
      Serial.println("Mission accomplished! System shutdown...");
      statusLEDs.showWhite(); // White indicates mission complete
      
      // Complete stop
      steeringServo.centerSteering();
      driveMotor.halt();
      
      Serial.println("Vehicle stopped successfully!");
      statusLEDs.flashPattern(0, 255, 0, 1000); // Victory celebration
      while (true) { delay(1000); } // Infinite hold
    }
  }
};

// ===== SYSTEM INITIALIZATION =====
AutonomousNavigator vehicleController;

// ===== MAIN PROGRAM EXECUTION =====
void setup() {
  if (!vehicleController.initializeAllSystems()) {
    Serial.println("Critical system failure during initialization!");
    while (true) { delay(1000); }
  }
  
  vehicleController.waitForStartSignal();
}

void loop() {
  vehicleController.executeNavigationCycle();
}
