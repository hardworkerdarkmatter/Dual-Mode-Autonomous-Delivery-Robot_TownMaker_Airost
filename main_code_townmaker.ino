#include <ESP32Servo.h>
#include "BluetoothSerial.h"

/*
   -----------------------------------------
   ESP32 ROBOT PIN MAP
   -----------------------------------------
   LEFT MOTOR:  ENA(21), IN1(15), IN2(19)
   RIGHT MOTOR: ENB(17), IN3(5),  IN4(4)
   SENSORS:     S1(25), S2(33), S3(32), S4(35), S5(34)
   ULTRASONIC:  TRIG(22), ECHO(23)
   SERVOS:      Shoulder(26), Elbow(14), Claw(27)
   -----------------------------------------
*/

BluetoothSerial SerialBT; 

// -------- PIN DEFINITIONS --------
#define LED_PIN 2       
#define BUTTON_PIN 13   

// --- MOTORS ---
#define ENA 21 
#define IN1 15
#define IN2 19
#define IN3 5
#define IN4 4
#define ENB 17

// --- SPEED SETTING ---
#define MOTOR_SPEED 160

// --- SENSORS ---
#define S1 25
#define S2 33
#define S3 32
#define S4 35
#define S5 34

// --- SENSOR LOGIC ---
#define LINE_DETECTED LOW

// --- ULTRASONIC ---
#define TRIG_PIN 22
#define ECHO_PIN 23
#define STOP_DISTANCE 5  

// --- SERVOS ---
#define PIN_SHOULDER 26
#define PIN_ELBOW    14
#define PIN_CLAW     27

// -------- SERVO ANGLES --------
int shoulderHome = 90;   
int elbowHome = 150;     // HIGH Angle for lifting UP
int elbowReach = 45;     // LOW Angle for going DOWN
int clawOpen = 40;       
int clawClosed = 90;     

Servo sShoulder;
Servo sElbow;
Servo sClaw;

// -------- SYSTEM STATE --------
bool manualMode = false;
bool hasObject = false; 
bool isArmDown = false; 

// MEMORY FOR SEARCHING
// -1 = Left, 1 = Right, 0 = Straight
int lastKnownDirection = 0; 

int lastButtonState = HIGH; 
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Robot"); 

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); 

  pinMode(S1, INPUT); pinMode(S2, INPUT); pinMode(S3, INPUT); 
  pinMode(S4, INPUT); pinMode(S5, INPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  sShoulder.attach(PIN_SHOULDER);
  sElbow.attach(PIN_ELBOW);
  sClaw.attach(PIN_CLAW);
  sShoulder.write(shoulderHome);
  sElbow.write(elbowHome);
  sClaw.write(clawClosed); 
  
  Serial.println("System Ready.");
}

// -------- MOVEMENT WITH SPEED CONTROL --------
void moveForward() {
  analogWrite(ENA, MOTOR_SPEED); analogWrite(ENB, MOTOR_SPEED); 
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void moveBackward() {
  analogWrite(ENA, MOTOR_SPEED); analogWrite(ENB, MOTOR_SPEED);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

// --- TURNING LOGIC ---
void turnLeft() {
  analogWrite(ENA, MOTOR_SPEED); analogWrite(ENB, MOTOR_SPEED);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);   // Left Stop
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);  // Right Fwd
}

void turnRight() {
  analogWrite(ENA, MOTOR_SPEED); analogWrite(ENB, MOTOR_SPEED);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);  // Left Fwd
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);   // Right Stop
}

void spinLeft() {
  analogWrite(ENA, MOTOR_SPEED); analogWrite(ENB, MOTOR_SPEED);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); // Left Back
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); // Right Fwd
}

void spinRight() {
  analogWrite(ENA, MOTOR_SPEED); analogWrite(ENB, MOTOR_SPEED);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); // Left Fwd
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); // Right Back
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

// -------- ROBOTIC ARM HELPER FUNCTIONS --------

void armDown() {
    Serial.println("Manual: Arm Down (Fast)");
    sElbow.write(elbowReach); // Instant move to 45
    sShoulder.write(shoulderHome); // Reset shoulder to 90
}

void armUp() {
    Serial.println("Manual: Arm Up (Fast)");
    // Shoulder moves to 50 (Up/Back) to help lift
    sShoulder.write(50); 
    sElbow.write(elbowHome);  // Instant move to 150
}

void pickupBall() {
  stopMotors();
  delay(500); 

  sClaw.write(clawOpen);
  delay(500);

  Serial.println("Elbow moving DOWN...");
  sElbow.write(elbowReach); // Fast move down
  delay(500); 

  sClaw.write(clawClosed);
  delay(1000);

  Serial.println("Elbow returning UP...");
  
  // Ensure Shoulder is Home
  sShoulder.write(shoulderHome); 
  
  // Lift Elbow Fast to 150
  sElbow.write(elbowHome);
  
  delay(1000);
}

// -------- AUTO MODE --------
void runAutoMode() {
  // 1. Ultrasonic Check
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
  long dist = duration * 0.034 / 2;

  // 2. Read Sensors
  int s1 = digitalRead(S1); 
  int s2 = digitalRead(S2); 
  int s3 = digitalRead(S3); 
  int s4 = digitalRead(S4); 
  int s5 = digitalRead(S5);

  // Track time
  static unsigned long lastTimeLineSeen = 0;
  if (s1 == LINE_DETECTED || s2 == LINE_DETECTED || s3 == LINE_DETECTED || s4 == LINE_DETECTED || s5 == LINE_DETECTED) {
    lastTimeLineSeen = millis();
  }

  // 3. Object Detection
  if (dist < STOP_DISTANCE && dist > 0 && hasObject == false) {
    Serial.println("!!! OBJECT DETECTED !!!");
    pickupBall();
    hasObject = true; 
  }

  // 4. Line Follower Logic
  if (s3 == LINE_DETECTED) {
    moveForward();
    lastKnownDirection = 0; 
  }
  else if (s1 == LINE_DETECTED || s2 == LINE_DETECTED) {
    spinRight(); 
    lastKnownDirection = -1;
  }
  else if (s4 == LINE_DETECTED || s5 == LINE_DETECTED) {
    spinLeft(); 
    lastKnownDirection = 1;
  }
  else {
    unsigned long lostDuration = millis() - lastTimeLineSeen;

    if (lostDuration < 2000) {
        static unsigned long lastShakeTime = 0;
        static int shakeDirection = 1; 
        if (millis() - lastShakeTime > 500) {
          shakeDirection = -shakeDirection;
          lastShakeTime = millis();
        }
        if (shakeDirection == 1) spinLeft();
        else spinRight();
    }
    else {
        moveBackward();
    }
  }
  
  delay(10);
}

// -------- MANUAL MODE --------
void runManualMode() {
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    
    // Movement
    if (cmd == 'F') moveForward();
    else if (cmd == 'B') moveBackward();
    else if (cmd == 'L') turnLeft();
    else if (cmd == 'R') turnRight();
    else if (cmd == 'S') stopMotors();
    
    // Arm Control
    else if (cmd == 'X') pickupBall();         
    else if (cmd == 'O') sClaw.write(clawOpen);    
    else if (cmd == 'C') sClaw.write(clawClosed);  
    
    // Toggle Arm
    else if (cmd == 'Z') {
      if (isArmDown) {
        armUp();
        isArmDown = false;
      } else {
        armDown();
        isArmDown = true;
      }
    }
  }
}

void loop() {
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 500) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    lastBlink = millis();
  }

  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    static int currentStableState = HIGH;
    if (reading != currentStableState) {
      currentStableState = reading;
      if (currentStableState == LOW) {
        manualMode = !manualMode;
        stopMotors();
        if(manualMode) Serial.println("Mode: MANUAL");
        else Serial.println("Mode: AUTO");
      }
    }
  }
  lastButtonState = reading;

  if (manualMode) {
    runManualMode();
  } else {
    runAutoMode();
  }
}
