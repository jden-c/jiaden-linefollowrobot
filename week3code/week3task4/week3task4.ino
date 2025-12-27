#include <SoftwareSerial.h>

// --- BLUETOOTH CONFIGURATION ---
// RX = A4 (Connect to HC-05 TX)
// TX = A5 (Connect to HC-05 RX)
SoftwareSerial btSerial(A4, A5); 

// --- MOTOR PINS (Your Setup) ---
#define MOTOR_A_EN 3   
#define MOTOR_A_IN1 1  
#define MOTOR_A_IN2 2  

#define MOTOR_B_EN 11    
#define MOTOR_B_IN1 12   
#define MOTOR_B_IN2 13   // WARNING: Pin 1 is Hardware TX. 
                        // If upload fails, unplug Pin 1.

// --- SETTINGS ---
int carSpeed = 255;     // Max Speed
char command = 'S';     // Status

void setup() {
  // 1. Setup Bluetooth on A4/A5
  btSerial.begin(9600); 

  // 2. Setup Motor Pins
  pinMode(MOTOR_A_EN, OUTPUT); pinMode(MOTOR_A_IN1, OUTPUT); pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_EN, OUTPUT); pinMode(MOTOR_B_IN1, OUTPUT); pinMode(MOTOR_B_IN2, OUTPUT);

  stopRobot();
}

void loop() {
  if (btSerial.available() > 0) {
    command = btSerial.read();
    processCommand(command);
  }
}

void processCommand(char cmd) {
  switch (cmd) {
    case 'F': moveForward(); break;
    case 'B': moveBackward(); break;
    case 'L': turnLeft(); break;
    case 'R': turnRight(); break;
    case 'S': stopRobot(); break;
    
    // Speed Settings (0, 1, ... 9, q)
    case '0': carSpeed = 0; break;
    case '5': carSpeed = 150; break;
    case '9': carSpeed = 255; break;
    case 'q': carSpeed = 255; break;
  }
}

// --- MOVEMENT FUNCTIONS ---

void moveForward() {
  digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW); analogWrite(MOTOR_A_EN, carSpeed);
  digitalWrite(MOTOR_B_IN1, HIGH); digitalWrite(MOTOR_B_IN2, LOW); analogWrite(MOTOR_B_EN, carSpeed);
}

void moveBackward() {
  digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, HIGH); analogWrite(MOTOR_A_EN, carSpeed);
  digitalWrite(MOTOR_B_IN1, LOW); digitalWrite(MOTOR_B_IN2, HIGH); analogWrite(MOTOR_B_EN, carSpeed);
}

void turnLeft() {
  digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW); analogWrite(MOTOR_A_EN, carSpeed);
  digitalWrite(MOTOR_B_IN1, LOW); digitalWrite(MOTOR_B_IN2, HIGH); analogWrite(MOTOR_B_EN, carSpeed);
}

void turnRight() {
  digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, HIGH); analogWrite(MOTOR_A_EN, carSpeed);
  digitalWrite(MOTOR_B_IN1, HIGH); digitalWrite(MOTOR_B_IN2, LOW); analogWrite(MOTOR_B_EN, carSpeed);
}

void stopRobot() {
  digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, LOW); analogWrite(MOTOR_A_EN, 0);
  digitalWrite(MOTOR_B_IN1, LOW); digitalWrite(MOTOR_B_IN2, LOW); analogWrite(MOTOR_B_EN, 0);
}
