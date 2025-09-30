/*
 * WRO Future Engineers Robot Controller
 * Complete implementation for 3 laps with 12 turns and self-centering
 * 
 * Hardware Components:
 * - Arduino Mega 2560
 * - TCS3200 Color Sensor (for turn direction detection)
 * - 2x MPU6050 Gyroscopes (primary + backup for precise orientation)
 * - 1x Servo Motor (steering control)
 * - 1x DC Motor with TB6612FNG Motor Driver
 * - 3x Ultrasonic Sensors (left, right, front wall detection)
 * - Rotary Encoder (distance measurement)
 * - Start Button and Status LED
 * 
 * Pin Configuration:
 * - TCS3200: S0=23, S1=25, S2=27, S3=29, OUT=31
 * - MPU6050 Primary: I2C Address 0x68 (SDA=20, SCL=21)
 * - MPU6050 Backup: I2C Address 0x69 (SDA=20, SCL=21)
 * - TB6612FNG: PWMA=10, ENA=9, ENB=11
 * - Servo: PWM=5
 * - Ultrasonic Left: TRIG=13, ECHO=A11
 * - Ultrasonic Right: TRIG=12, ECHO=A12
 * - Ultrasonic Front: TRIG=4, ECHO=3
 * - Encoder: A=22, B=24
 * - Button: A0, LED: 2
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// ===== PIN DEFINITIONS =====
// TCS3200 Color Sensor
#define TCS_S0_PIN    23
#define TCS_S1_PIN    25
#define TCS_S2_PIN    27
#define TCS_S3_PIN    29
#define TCS_OUT_PIN   31

// TB6612FNG Motor Driver
#define MOTOR_PWMA_PIN   10
#define MOTOR_ENA_PIN    9
#define MOTOR_ENB_PIN    11

// Servo Motor
#define SERVO_PIN     5

// Ultrasonic Sensors
#define US_LEFT_TRIG  13
#define US_LEFT_ECHO  A11
#define US_RIGHT_TRIG 12
#define US_RIGHT_ECHO A12
#define US_FRONT_TRIG 4
#define US_FRONT_ECHO 3

// NeoPixel Status Display
#define NEOPIXEL_PIN  8
#define NEOPIXEL_COUNT 2

// Control Interface
#define BUTTON_PIN    A0

// MPU6050 I2C Addresses
#define MPU_PRIMARY_ADDR   0x68
#define MPU_BACKUP_ADDR    0x69

// ===== TCS3200 COLOR SENSOR =====
class TCS3200ColorSensor {
private:
  // Calibrated thresholds for track colors
  struct ColorProfile {
    int redThreshold;
    int greenThreshold;
    int blueThreshold;
    String name;
  };
  
  ColorProfile blueProfile = {150, 150, 80, "BLUE"};    // Blue pillar
  ColorProfile orangeProfile = {80, 120, 150, "ORANGE"}; // Orange pillar
  
public:
  enum DetectedColor { NONE, BLUE, ORANGE };
  
  void init() {
    pinMode(TCS_S0_PIN, OUTPUT);
    pinMode(TCS_S1_PIN, OUTPUT);
    pinMode(TCS_S2_PIN, OUTPUT);
    pinMode(TCS_S3_PIN, OUTPUT);
    pinMode(TCS_OUT_PIN, INPUT);
    
    // Set frequency scaling to 20% for optimal reading
    digitalWrite(TCS_S0_PIN, HIGH);
    digitalWrite(TCS_S1_PIN, LOW);
    
    Serial.println("TCS3200 Color Sensor initialized");
  }
  
  int readColorFrequency(char filter) {
    switch(filter) {
      case 'R': // Red filter
        digitalWrite(TCS_S2_PIN, LOW);
        digitalWrite(TCS_S3_PIN, LOW);
        break;
      case 'G': // Green filter
        digitalWrite(TCS_S2_PIN, HIGH);
        digitalWrite(TCS_S3_PIN, HIGH);
        break;
      case 'B': // Blue filter
        digitalWrite(TCS_S2_PIN, LOW);
        digitalWrite(TCS_S3_PIN, HIGH);
        break;
      case 'C': // Clear (no filter)
        digitalWrite(TCS_S2_PIN, HIGH);
        digitalWrite(TCS_S3_PIN, LOW);
        break;
    }
    
    delayMicroseconds(100);
    return pulseIn(TCS_OUT_PIN, LOW, 50000);
  }
  
  DetectedColor detectColor() {
    int red = readColorFrequency('R');
    int green = readColorFrequency('G');
    int blue = readColorFrequency('B');
    int clear = readColorFrequency('C');
    
    // Avoid division by zero
    if (clear == 0) return NONE;
    
    // Normalize to percentage
    float redRatio = (float)red / clear * 100;
    float greenRatio = (float)green / clear * 100;
    float blueRatio = (float)blue / clear * 100;
    
    // Debug output
    Serial.print("Color Ratios - R:");
    Serial.print(redRatio, 1);
    Serial.print(" G:");
    Serial.print(greenRatio, 1);
    Serial.print(" B:");
    Serial.print(blueRatio, 1);
    
    // Detect blue pillar (blue dominates)
    if (blueRatio < blueProfile.blueThreshold && 
        redRatio > blueProfile.redThreshold && 
        greenRatio > blueProfile.greenThreshold) {
      Serial.println(" -> BLUE detected");
      return BLUE;
    }
    
    // Detect orange pillar (red+green dominate, blue low)
    if (redRatio < orangeProfile.redThreshold && 
        greenRatio < orangeProfile.greenThreshold && 
        blueRatio > orangeProfile.blueThreshold) {
      Serial.println(" -> ORANGE detected");
      return ORANGE;
    }
    
    Serial.println(" -> No pillar");
    return NONE;
  }
};

// ===== DUAL MPU6050 GYROSCOPE SYSTEM =====
class DualGyroSystem {
private:
  Adafruit_MPU6050 primaryMPU;
  Adafruit_MPU6050 backupMPU;
  
  float yaw;
  float targetYaw;
  float primaryOffset;
  float backupOffset;
  float previousError;
  bool primaryActive;
  bool backupActive;
  
  // PID Constants for orientation control
  const float Kp = 2.2;
  const float Ki = 0.1;
  const float Kd = 1.5;
  float integralError;
  
public:
  DualGyroSystem() : yaw(0), targetYaw(0), primaryOffset(0), backupOffset(0), 
                     previousError(0), primaryActive(false), backupActive(false),
                     integralError(0) {}
  
  bool init() {
    Wire.begin();
    
    // Initialize primary MPU6050
    if (primaryMPU.begin(MPU_PRIMARY_ADDR)) {
      primaryMPU.setAccelerometerRange(MPU6050_RANGE_8_G);
      primaryMPU.setGyroRange(MPU6050_RANGE_500_DEG);
      primaryMPU.setFilterBandwidth(MPU6050_BAND_21_HZ);
      primaryActive = true;
      Serial.println("Primary MPU6050 initialized");
    } else {
      Serial.println("Primary MPU6050 failed to initialize");
    }
    
    // Initialize backup MPU6050
    if (backupMPU.begin(MPU_BACKUP_ADDR)) {
      backupMPU.setAccelerometerRange(MPU6050_RANGE_8_G);
      backupMPU.setGyroRange(MPU6050_RANGE_500_DEG);
      backupMPU.setFilterBandwidth(MPU6050_BAND_21_HZ);
      backupActive = true;
      Serial.println("Backup MPU6050 initialized");
    } else {
      Serial.println("Backup MPU6050 failed to initialize");
    }
    
    if (!primaryActive && !backupActive) {
      Serial.println("ERROR: No gyroscopes available!");
      return false;
    }
    
    calibrateGyros();
    return true;
  }
  
  void calibrateGyros() {
    const int samples = 300;
    float primarySum = 0, backupSum = 0;
    
    Serial.println("Calibrating gyroscopes...");
    
    for (int i = 0; i < samples; i++) {
      if (primaryActive) {
        sensors_event_t a, g, temp;
        primaryMPU.getEvent(&a, &g, &temp);
        primarySum += g.gyro.z;
      }
      
      if (backupActive) {
        sensors_event_t a2, g2, temp2;
        backupMPU.getEvent(&a2, &g2, &temp2);
        backupSum += g2.gyro.z;
      }
      
      delay(5);
    }
    
    if (primaryActive) primaryOffset = primarySum / samples;
    if (backupActive) backupOffset = backupSum / samples;
    
    Serial.println("Gyroscope calibration complete");
  }
  
  void update(float deltaTime) {
    float gyroZ = 0;
    int activeCount = 0;
    
    // Read from primary gyro
    if (primaryActive) {
      sensors_event_t a, g, temp;
      if (primaryMPU.getEvent(&a, &g, &temp)) {
        gyroZ += (g.gyro.z - primaryOffset);
        activeCount++;
      } else {
        primaryActive = false;
        Serial.println("Primary gyro failed, switching to backup");
      }
    }
    
    // Read from backup gyro
    if (backupActive) {
      sensors_event_t a2, g2, temp2;
      if (backupMPU.getEvent(&a2, &g2, &temp2)) {
        gyroZ += (g2.gyro.z - backupOffset);
        activeCount++;
      } else {
        backupActive = false;
        Serial.println("Backup gyro failed");
      }
    }
    
    if (activeCount == 0) {
      Serial.println("ERROR: All gyroscopes failed!");
      return;
    }
    
    // Average the readings
    gyroZ /= activeCount;
    
    // Convert to degrees/sec and integrate
    gyroZ *= 57.2958; // radians to degrees
    
    if (abs(gyroZ) > 0.2) { // Noise threshold
      yaw += gyroZ * deltaTime;
    }
  }
  
  int calculateSteeringCorrection() {
    float error = targetYaw - yaw;
    
    // Normalize error to [-180, 180]
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    
    integralError += error;
    integralError = constrain(integralError, -100, 100); // Prevent windup
    
    float derivative = error - previousError;
    previousError = error;
    
    float correction = Kp * error + Ki * integralError + Kd * derivative;
    return (int)constrain(correction, -40, 40); // Limit steering range
  }
  
  void setTargetYaw(float target) { 
    targetYaw = target;
    integralError = 0; // Reset integral term
  }
  
  void adjustTargetYaw(float adjustment) {
    targetYaw += adjustment;
  }
  
  float getYaw() { return yaw; }
  float getTargetYaw() { return targetYaw; }
  float getError() { return abs(targetYaw - yaw); }
  bool isSystemHealthy() { return primaryActive || backupActive; }
};

// ===== NEOPIXEL STATUS DISPLAY =====
class NeoPixelStatusDisplay {
private:
  Adafruit_NeoPixel strip;
  unsigned long lastUpdate;
  int brightness;
  int fadeDirection;
  bool signaling;
  int signalType; // 0=off, 1=ready, 2=driving, 3=turning, 4=centering, 5=finished, 6=error
  
  const int FADE_SPEED = 8;
  const int MAX_BRIGHTNESS = 255;
  const int MIN_BRIGHTNESS = 20;
  
public:
  NeoPixelStatusDisplay() : 
    strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800),
    lastUpdate(0), brightness(MIN_BRIGHTNESS), fadeDirection(1),
    signaling(false), signalType(0) {}
  
  void init() {
    strip.begin();
    strip.show();
    strip.setBrightness(255);
    Serial.println("NeoPixel Status Display initialized");
  }
  
  void setStatus(int status) {
    signalType = status;
    signaling = (status != 0);
    brightness = MIN_BRIGHTNESS;
    fadeDirection = 1;
  }
  
  void off() {
    signaling = false;
    signalType = 0;
    strip.clear();
    strip.show();
  }
  
  void update() {
    if (!signaling) return;
    
    unsigned long now = millis();
    if (now - lastUpdate < 30) return; // ~33Hz update rate
    
    lastUpdate = now;
    
    // Update brightness with fade effect
    brightness += fadeDirection * FADE_SPEED;
    
    if (brightness >= MAX_BRIGHTNESS) {
      brightness = MAX_BRIGHTNESS;
      fadeDirection = -1;
    } else if (brightness <= MIN_BRIGHTNESS) {
      brightness = MIN_BRIGHTNESS;
      fadeDirection = 1;
    }
    
    uint32_t color = getStatusColor();
    
    for (int i = 0; i < NEOPIXEL_COUNT; i++) {
      strip.setPixelColor(i, color);
    }
    strip.show();
  }
  
  void flash(uint8_t r, uint8_t g, uint8_t b, int times = 3) {
    for (int i = 0; i < times; i++) {
      for (int j = 0; j < NEOPIXEL_COUNT; j++) {
        strip.setPixelColor(j, strip.Color(r, g, b));
      }
      strip.show();
      delay(200);
      
      strip.clear();
      strip.show();
      delay(200);
    }
  }
  
  void rainbow(int duration = 50) {
    for (int j = 0; j < 256; j++) {
      for (int i = 0; i < NEOPIXEL_COUNT; i++) {
        strip.setPixelColor(i, wheelColor((j + i * 256 / NEOPIXEL_COUNT) & 255));
      }
      strip.show();
      delay(duration);
    }
  }
  
private:
  uint32_t getStatusColor() {
    uint8_t r = 0, g = 0, b = 0;
    
    switch (signalType) {
      case 1: // Ready - Green
        g = brightness;
        break;
      case 2: // Driving - Blue
        b = brightness;
        break;
      case 3: // Turning - Orange
        r = brightness;
        g = brightness / 2;
        break;
      case 4: // Centering - Purple
        r = brightness;
        b = brightness;
        break;
      case 5: // Finished - White
        r = brightness;
        g = brightness;
        b = brightness;
        break;
      case 6: // Error - Red
        r = brightness;
        break;
      default:
        break;
    }
    
    return strip.Color(r, g, b);
  }
  
  uint32_t wheelColor(byte wheelPos) {
    wheelPos = 255 - wheelPos;
    if (wheelPos < 85) {
      return strip.Color(255 - wheelPos * 3, 0, wheelPos * 3);
    }
    if (wheelPos < 170) {
      wheelPos -= 85;
      return strip.Color(0, wheelPos * 3, 255 - wheelPos * 3);
    }
    wheelPos -= 170;
    return strip.Color(wheelPos * 3, 255 - wheelPos * 3, 0);
  }
};

// ===== TB6612FNG MOTOR CONTROLLER =====
class TB6612MotorController {
private:
  int pwmPin, enableA, enableB;
  int currentSpeed;
  bool isMoving;
  
public:
  TB6612MotorController(int pwm, int enA, int enB) : 
    pwmPin(pwm), enableA(enA), enableB(enB), currentSpeed(0), isMoving(false) {}
  
  void init() {
    pinMode(pwmPin, OUTPUT);
    pinMode(enableA, OUTPUT);
    pinMode(enableB, OUTPUT);
    stop();
    Serial.println("TB6612FNG Motor Controller initialized");
  }
  
  void moveForward(int speed) {
    speed = constrain(speed, 0, 255);
    currentSpeed = speed;
    isMoving = (speed > 0);
    
    // Forward: ENA=HIGH, ENB=LOW
    digitalWrite(enableA, HIGH);
    digitalWrite(enableB, LOW);
    analogWrite(pwmPin, speed);
  }
  
  void stop() {
    // Coast stop: both pins LOW
    digitalWrite(enableA, LOW);
    digitalWrite(enableB, LOW);
    analogWrite(pwmPin, 0);
    currentSpeed = 0;
    isMoving = false;
  }
  
  void brake() {
    // Hard brake: both pins HIGH
    digitalWrite(enableA, HIGH);
    digitalWrite(enableB, HIGH);
    analogWrite(pwmPin, 0);
    delay(100); // Brief brake
    stop(); // Then coast
  }
  
  int getSpeed() { return currentSpeed; }
  bool getMovementStatus() { return isMoving; }
};

// ===== SERVO STEERING CONTROLLER =====
class ServoSteeringController {
private:
  Servo steeringServo;
  int currentAngle;
  
  const int STRAIGHT_ANGLE = 90;
  const int LEFT_LIMIT = 50;
  const int RIGHT_LIMIT = 130;
  
public:
  void init() {
    steeringServo.attach(SERVO_PIN);
    currentAngle = STRAIGHT_ANGLE;
    setStraight();
    delay(500);
    Serial.println("Servo Steering initialized");
  }
  
  void setAngle(int angle) {
    angle = constrain(angle, LEFT_LIMIT, RIGHT_LIMIT);
    currentAngle = angle;
    steeringServo.write(angle);
  }
  
  void setStraight() {
    setAngle(STRAIGHT_ANGLE);
  }
  
  void applyCorrection(int correction) {
    setAngle(STRAIGHT_ANGLE + correction);
  }
  
  int getAngle() { return currentAngle; }
};

// ===== ULTRASONIC SENSOR ARRAY =====
class UltrasonicSensorArray {
private:
  struct SensorPins {
    int trigPin, echoPin;
  };
  
  SensorPins leftSensor = {US_LEFT_TRIG, US_LEFT_ECHO};
  SensorPins rightSensor = {US_RIGHT_TRIG, US_RIGHT_ECHO};
  SensorPins frontSensor = {US_FRONT_TRIG, US_FRONT_ECHO};
  
  int measureDistance(SensorPins sensor) {
    digitalWrite(sensor.trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(sensor.trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(sensor.trigPin, LOW);
    
    long duration = pulseIn(sensor.echoPin, HIGH, 30000);
    if (duration == 0) return 0;
    
    return duration * 0.034 / 2; // Convert to cm
  }
  
  int getFilteredDistance(SensorPins sensor) {
    const int numSamples = 3;
    int samples[numSamples];
    int validCount = 0;
    
    for (int i = 0; i < numSamples; i++) {
      int distance = measureDistance(sensor);
      if (distance > 2 && distance < 100) {
        samples[validCount++] = distance;
      }
      delay(10);
    }
    
    if (validCount == 0) return 0;
    
    // Return median
    for (int i = 0; i < validCount - 1; i++) {
      for (int j = i + 1; j < validCount; j++) {
        if (samples[i] > samples[j]) {
          int temp = samples[i];
          samples[i] = samples[j];
          samples[j] = temp;
        }
      }
    }
    
    return samples[validCount / 2];
  }
  
public:
  void init() {
    pinMode(leftSensor.trigPin, OUTPUT);
    pinMode(leftSensor.echoPin, INPUT);
    pinMode(rightSensor.trigPin, OUTPUT);
    pinMode(rightSensor.echoPin, INPUT);
    pinMode(frontSensor.trigPin, OUTPUT);
    pinMode(frontSensor.echoPin, INPUT);
    
    Serial.println("Ultrasonic Sensor Array initialized");
  }
  
  int getLeftDistance() { return getFilteredDistance(leftSensor); }
  int getRightDistance() { return getFilteredDistance(rightSensor); }
  int getFrontDistance() { return getFilteredDistance(frontSensor); }
  
  bool isWallDetected(int distance) {
    return (distance > 5 && distance < 80);
  }
};

// ===== MAIN ROBOT CONTROLLER =====
class WROFutureEngineersRobot {
private:
  // Component objects
  TCS3200ColorSensor colorSensor;
  DualGyroSystem gyroSystem;
  TB6612MotorController motor;
  ServoSteeringController steering;
  UltrasonicSensorArray ultrasonics;
  NeoPixelStatusDisplay statusDisplay;
  
  // State management
  enum RobotState { WAITING, DRIVING, TURNING, CENTERING, FINISHED };
  RobotState currentState;
  
  // Race tracking
  int turnCount;
  int turnDirection; // -1 = left, 1 = right, 0 = undefined
  bool directionEstablished;
  
  // Timing
  unsigned long lastUpdateTime;
  unsigned long lastCenteringTime;
  unsigned long turnStartTime;
  
  // Speed settings
  const int NORMAL_SPEED = 80;
  const int TURN_SPEED = 120;
  const int SLOW_SPEED = 60;
  
  // Distance thresholds
  const int FRONT_WALL_THRESHOLD = 45; // cm
  const int SIDE_WALL_TARGET = 25;     // cm
  const int CENTERING_TOLERANCE = 8;   // cm
  
public:
  WROFutureEngineersRobot() : 
    motor(MOTOR_PWMA_PIN, MOTOR_ENA_PIN, MOTOR_ENB_PIN),
    currentState(WAITING),
    turnCount(0),
    turnDirection(0),
    directionEstablished(false),
    lastUpdateTime(0),
    lastCenteringTime(0),
    turnStartTime(0) {}
  
  bool initialize() {
    Serial.begin(115200);
    Serial.println("=================================");
    Serial.println("WRO Future Engineers Robot");
    Serial.println("3 Laps, 12 Turns, Self-Centering");
    Serial.println("=================================");
    
    // Initialize all components
    colorSensor.init();
    
    if (!gyroSystem.init()) {
      Serial.println("CRITICAL: Gyroscope system failed!");
      return false;
    }
    
    motor.init();
    steering.init();
    ultrasonics.init();
    statusDisplay.init();
    
    // Setup control pins
    pinMode(BUTTON_PIN, INPUT);
    
    Serial.println("Robot initialization complete!");
    return true;
  }
  
  void waitForStart() {
    Serial.println("\nPress button to start race...");
    statusDisplay.setStatus(1); // Ready status - green
    
    while (!digitalRead(BUTTON_PIN)) {
      statusDisplay.update();
      delay(10);
    }
    
    statusDisplay.setStatus(2); // Driving status - blue
    Serial.println("RACE STARTED!");
    
    currentState = DRIVING;
    lastUpdateTime = millis();
  }
  
  void run() {
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime < 20) return; // 50Hz update rate
    
    float deltaTime = (currentTime - lastUpdateTime) / 1000.0;
    lastUpdateTime = currentTime;
    
    // Update gyroscope system
    gyroSystem.update(deltaTime);
    
    // Update status display
    statusDisplay.update();
    
    // State machine
    switch (currentState) {
      case WAITING:
        // Do nothing, wait for start
        break;
        
      case DRIVING:
        handleDriving();
        break;
        
      case TURNING:
        handleTurning();
        break;
        
      case CENTERING:
        handleCentering();
        break;
        
      case FINISHED:
        handleFinished();
        break;
    }
    
    // Check for race completion
    if (turnCount >= 12 && currentState != FINISHED) {
      Serial.println("RACE COMPLETED! 3 laps finished!");
      currentState = FINISHED;
    }
  }
  
private:
  void handleDriving() {
    motor.moveForward(NORMAL_SPEED);
    statusDisplay.setStatus(2); // Driving status - blue
    
    // Apply gyroscope-based steering correction
    int gyroCorrection = gyroSystem.calculateSteeringCorrection();
    steering.applyCorrection(gyroCorrection);
    
    // Check for turn direction establishment (first turn only)
    if (!directionEstablished) {
      TCS3200ColorSensor::DetectedColor detectedColor = colorSensor.detectColor();
      if (detectedColor != TCS3200ColorSensor::NONE) {
        establishTurnDirection(detectedColor);
        return;
      }
    }
    
    // Check for front wall (turn trigger)
    int frontDistance = ultrasonics.getFrontDistance();
    if (ultrasonics.isWallDetected(frontDistance) && frontDistance < FRONT_WALL_THRESHOLD) {
      initiateTurn();
      return;
    }
    
    // Perform wall centering
    performWallCentering();
  }
  
  void establishTurnDirection(TCS3200ColorSensor::DetectedColor color) {
    turnDirection = (color == TCS3200ColorSensor::BLUE) ? -1 : 1;
    directionEstablished = true;
    
    Serial.print("Turn direction established: ");
    Serial.println(turnDirection == -1 ? "LEFT (Blue detected)" : "RIGHT (Orange detected)");
    Serial.println("Future turns will be based on front wall detection");
  }
  
  void initiateTurn() {
    if (!directionEstablished) {
      Serial.println("Cannot turn - direction not established yet");
      return;
    }
    
    turnCount++;
    Serial.print("Initiating turn #");
    Serial.print(turnCount);
    Serial.println(turnDirection == -1 ? " (LEFT)" : " (RIGHT)");
    
    // Set new target yaw
    float yawChange = 90.0 * turnDirection;
    gyroSystem.adjustTargetYaw(yawChange);
    
    motor.moveForward(TURN_SPEED);
    currentState = TURNING;
    turnStartTime = millis();
    
    statusDisplay.setStatus(3); // Turning status - orange
  }
  
  void handleTurning() {
    motor.moveForward(TURN_SPEED);
    
    // Apply gyroscope correction during turn
    int gyroCorrection = gyroSystem.calculateSteeringCorrection();
    steering.applyCorrection(gyroCorrection);
    
    // Check if turn is complete
    if (gyroSystem.getError() < 5.0) {
      Serial.println("Turn completed");
      currentState = CENTERING;
      statusDisplay.setStatus(4); // Centering status - purple
      lastCenteringTime = millis();
    }
    
    // Safety timeout for turns
    if (millis() - turnStartTime > 5000) {
      Serial.println("Turn timeout - continuing");
      currentState = DRIVING;
      statusDisplay.setStatus(2); // Back to driving status
    }
  }
  
  void handleCentering() {
    motor.moveForward(SLOW_SPEED);
    statusDisplay.setStatus(4); // Centering status - purple
    
    // Apply gyroscope-based steering correction
    int gyroCorrection = gyroSystem.calculateSteeringCorrection();
    steering.applyCorrection(gyroCorrection);
    
    // Perform intensive centering for a short period
    performWallCentering();
    
    // Return to normal driving after centering period
    if (millis() - lastCenteringTime > 2000) {
      currentState = DRIVING;
      statusDisplay.setStatus(2); // Back to driving status
      Serial.println("Centering complete, resuming normal driving");
    }
  }
  
  void performWallCentering() {
    int leftDistance = ultrasonics.getLeftDistance();
    int rightDistance = ultrasonics.getRightDistance();
    
    // Only perform centering if both walls are detected
    if (!ultrasonics.isWallDetected(leftDistance) || !ultrasonics.isWallDetected(rightDistance)) {
      return;
    }
    
    int distanceDifference = leftDistance - rightDistance;
    
    if (abs(distanceDifference) > CENTERING_TOLERANCE) {
      float centeringAdjustment;
      
      if (distanceDifference > 0) {
        // Too close to right wall, steer left
        centeringAdjustment = -3.0;
      } else {
        // Too close to left wall, steer right
        centeringAdjustment = 3.0;
      }
      
      gyroSystem.adjustTargetYaw(centeringAdjustment);
      
      Serial.print("Centering: L=");
      Serial.print(leftDistance);
      Serial.print("cm R=");
      Serial.print(rightDistance);
      Serial.print("cm Diff=");
      Serial.print(distanceDifference);
      Serial.print("cm Adj=");
      Serial.println(centeringAdjustment);
    }
  }
  
  void handleFinished() {
    // Gradually slow down and stop
    static int finishSpeed = NORMAL_SPEED;
    static unsigned long lastSlowDown = 0;
    
    if (millis() - lastSlowDown > 200) {
      finishSpeed = max(0, finishSpeed - 10);
      motor.moveForward(finishSpeed);
      lastSlowDown = millis();
      
      if (finishSpeed == 0) {
        motor.stop();
        steering.setStraight();
        
        Serial.println("================================");
        Serial.println("RACE FINISHED SUCCESSFULLY!");
        Serial.println("Robot stopped.");
        Serial.println("================================");
        
        // Victory light show
        statusDisplay.setStatus(5); // Finished status - white
        delay(1000);
        statusDisplay.rainbow(30); // Rainbow celebration
        statusDisplay.flash(0, 255, 0, 5); // Green victory flashes
        statusDisplay.setStatus(5); // Back to finished status
      }
    }
  }
  
public:
  void emergencyStop() {
    motor.brake();
    steering.setStraight();
    currentState = FINISHED;
    statusDisplay.setStatus(6); // Error status - red
    Serial.println("EMERGENCY STOP ACTIVATED!");
  }
  
  void printStatus() {
    Serial.println("=== Robot Status ===");
    Serial.print("State: ");
    
    switch (currentState) {
      case WAITING: Serial.println("WAITING"); break;
      case DRIVING: Serial.println("DRIVING"); break;
      case TURNING: Serial.println("TURNING"); break;
      case CENTERING: Serial.println("CENTERING"); break;
      case FINISHED: Serial.println("FINISHED"); break;
    }
    
    Serial.print("Turns completed: ");
    Serial.print(turnCount);
    Serial.println("/12");
    
    Serial.print("Turn direction: ");
    if (!directionEstablished) {
      Serial.println("Not established");
    } else {
      Serial.println(turnDirection == -1 ? "LEFT" : "RIGHT");
    }
    
    Serial.print("Yaw: ");
    Serial.print(gyroSystem.getYaw(), 1);
    Serial.print("° Target: ");
    Serial.print(gyroSystem.getTargetYaw(), 1);
    Serial.print("° Error: ");
    Serial.print(gyroSystem.getError(), 1);
    Serial.println("°");
    
    Serial.print("Ultrasonic - L:");
    Serial.print(ultrasonics.getLeftDistance());
    Serial.print("cm R:");
    Serial.print(ultrasonics.getRightDistance());
    Serial.print("cm F:");
    Serial.print(ultrasonics.getFrontDistance());
    Serial.println("cm");
    
    Serial.print("Motor speed: ");
    Serial.println(motor.getSpeed());
    
    Serial.print("Steering angle: ");
    Serial.println(steering.getAngle());
    
    Serial.print("Gyro system health: ");
    Serial.println(gyroSystem.isSystemHealthy() ? "OK" : "DEGRADED");
    
    Serial.println("===================");
  }
};

// ===== GLOBAL ROBOT INSTANCE =====
WROFutureEngineersRobot robot;

// ===== MAIN ARDUINO FUNCTIONS =====
void setup() {
  if (!robot.initialize()) {
    Serial.println("CRITICAL ERROR: Robot initialization failed!");
    // Create a temporary NeoPixel instance for error display
    Adafruit_NeoPixel errorStrip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
    errorStrip.begin();
    
    while (true) {
      // Flash red for error
      for (int i = 0; i < NEOPIXEL_COUNT; i++) {
        errorStrip.setPixelColor(i, errorStrip.Color(255, 0, 0));
      }
      errorStrip.show();
      delay(200);
      
      errorStrip.clear();
      errorStrip.show();
      delay(200);
    }
  }
  
  robot.waitForStart();
}

void loop() {
  robot.run();
  
  // Emergency stop check (hold button for 3 seconds)
  static unsigned long buttonHoldStart = 0;
  if (digitalRead(BUTTON_PIN)) {
    if (buttonHoldStart == 0) {
      buttonHoldStart = millis();
    } else if (millis() - buttonHoldStart > 3000) {
      robot.emergencyStop();
      buttonHoldStart = 0;
    }
  } else {
    buttonHoldStart = 0;
  }
  
  // Status report every 5 seconds
  static unsigned long lastStatusReport = 0;
  if (millis() - lastStatusReport > 5000) {
    robot.printStatus();
    lastStatusReport = millis();
  }
}
