#include <ESP32Servo.h>
#include "BluetoothSerial.h"

// -------- BLUETOOTH SETUP --------
BluetoothSerial SerialBT; 

// -------- PIN DEFINITIONS --------
#define LED_PIN 2       // Onboard Blue LED (Heartbeat)
#define BUTTON_PIN 13   // Push Button

// MOTORS
#define ENA 21 
#define IN1 15
#define IN2 19
#define IN3 5
#define IN4 4
#define ENB 17

// SENSORS
#define S1 25
#define S2 33
#define S3 32
#define S4 35
#define S5 34
#define LINE_DETECTED HIGH 

#define TRIG_PIN 22
#define ECHO_PIN 23
#define STOP_DISTANCE 15 

// SERVOS (Shoulder, Elbow, Claw only)
#define PIN_SHOULDER 26
#define PIN_ELBOW    14
#define PIN_CLAW     27

// -------- SERVO ANGLES --------
int shoulderHome = 90;   
int shoulderReach = 160; 
int elbowHome = 90;      
int elbowReach = 45;     
int clawOpen = 40;      
int clawClosed = 90;     

Servo sShoulder;
Servo sElbow;
Servo sClaw;

// -------- SYSTEM STATE --------
bool manualMode = false;
bool hasObject = false; 

// Button State Tracking
int lastButtonState = HIGH; 
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_Robot"); 

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP); 

  // Init Sensors & Motors
  pinMode(S1, INPUT); pinMode(S2, INPUT); pinMode(S3, INPUT); pinMode(S4, INPUT); pinMode(S5, INPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  // Init Servos
  sShoulder.attach(PIN_SHOULDER);
  sElbow.attach(PIN_ELBOW);
  sClaw.attach(PIN_CLAW);

  // Set Home
  sShoulder.write(shoulderHome);
  sElbow.write(elbowHome);
  sClaw.write(clawClosed); 
  
  Serial.println("System Ready. Blue LED should be blinking.");
}

// -------- MOVEMENT --------
void moveForward() {
  digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH); 
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}
void moveBackward() {
  digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}
void turnLeft() {
  digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
void turnRight() {
  digitalWrite(ENA, HIGH); digitalWrite(ENB, HIGH);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// -------- ROBOTIC ARM --------
void pickupBall() {
  stopMotors();
  delay(500); 

  sShoulder.write(shoulderReach);
  sElbow.write(elbowReach);
  delay(1000); 

  sClaw.write(clawOpen);
  delay(1000); 

  sClaw.write(clawClosed);
  delay(1000);

  sShoulder.write(shoulderHome);
  sElbow.write(elbowHome);
  delay(1000);
}

// -------- MODES --------
void runAutoMode() {
  // Ultrasonic Check
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  long dist = duration * 0.034 / 2;

  if (dist < STOP_DISTANCE && dist > 0 && hasObject == false) {
    pickupBall();
    hasObject = true; 
  }

  // Line Follower
  int s1 = digitalRead(S1); int s2 = digitalRead(S2); int s3 = digitalRead(S3); int s4 = digitalRead(S4); int s5 = digitalRead(S5);

  if (s1 == LINE_DETECTED && s2 == LINE_DETECTED && s3 == LINE_DETECTED && s4 == LINE_DETECTED && s5 == LINE_DETECTED) {
    stopMotors(); 
  }
  else if (s3 == LINE_DETECTED) moveForward();
  else if (s1 == LINE_DETECTED || s2 == LINE_DETECTED) turnLeft();
  else if (s4 == LINE_DETECTED || s5 == LINE_DETECTED) turnRight();
  else stopMotors();
  
  delay(10);
}

void runManualMode() {
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    if (cmd == 'F') moveForward();
    else if (cmd == 'B') moveBackward();
    else if (cmd == 'L') turnLeft();
    else if (cmd == 'R') turnRight();
    else if (cmd == 'S') stopMotors();
    else if (cmd == 'X') pickupBall(); 
    else if (cmd == 'O') sClaw.write(clawOpen);
    else if (cmd == 'C') sClaw.write(clawClosed);
  }
}

// -------- MAIN LOOP (Fixed) --------
void loop() {
  // 1. Heartbeat (Blink LED every loop)
  // This proves the ESP32 is NOT frozen
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 500) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    lastBlink = millis();
  }

  // 2. Button Check (State Change Detection)
  int reading = digitalRead(BUTTON_PIN);

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    lastDebounceTime = millis(); // reset the timer
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If state has been stable for 50ms, it's a real press
    // We only toggle if the button went from HIGH to LOW (Press)
    // We do NOT toggle if it is HELD LOW (Stuck)
    static int currentStableState = HIGH;
    
    if (reading != currentStableState) {
      currentStableState = reading;
      
      if (currentStableState == LOW) {
        // BUTTON WAS PRESSED!
        manualMode = !manualMode;
        stopMotors();
        
        if(manualMode) Serial.println("Mode: MANUAL (Bluetooth)");
        else Serial.println("Mode: AUTO (Line Follower)");
      }
    }
  }
  
  lastButtonState = reading;

  // 3. Run Mode
  if (manualMode) {
    runManualMode();
  } else {
    runAutoMode();
  }
}