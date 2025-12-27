#include <Wire.h>
#include <LiquidCrystal.h>

// --- MOTOR PINS ---
#define MOTOR_A_EN 11   
#define MOTOR_A_IN1 13  
#define MOTOR_A_IN2 12  

#define MOTOR_B_EN 3    
#define MOTOR_B_IN1 2   
#define MOTOR_B_IN2 1   // WARNING: Unplug Pin 1 to upload code

// --- SENSOR PINS ---
#define TRIG_PIN A3     
#define ECHO_PIN A2     

// --- SETTINGS ---
#define OBSTACLE_DISTANCE 30 // Increased detection range slightly (cm)
#define CRUISE_SPEED 150     // Speed when moving forward (0-255)
#define TURN_SPEED 240       // Faster speed for "Exaggerated" turning (0-255)
#define TURN_TIME 800        // How long to rotate in milliseconds (higher = longer turn)

void setup() {
  // Motor Pins
  pinMode(MOTOR_A_EN, OUTPUT); pinMode(MOTOR_A_IN1, OUTPUT); pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_EN, OUTPUT); pinMode(MOTOR_B_IN1, OUTPUT); pinMode(MOTOR_B_IN2, OUTPUT);

  // Sensor Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  delay(2000); 
}

void loop() {
  int distance = getDistance();

  // --- LOGIC ---
  if (distance > 0 && distance < OBSTACLE_DISTANCE) {
    // 1. Stop briefly to register the obstacle
    stopRobot();
    delay(200);

    // 2. Perform EXAGGERATED Turn
    // This function now runs for a specific time (TURN_TIME)
    rotateRobot(); 
    delay(TURN_TIME); // Force the robot to keep turning for this long
    
    // 3. Stop briefly to stabilize before checking again
    stopRobot();
    delay(200);
  } 
  else {
    moveForward();
  }
  
  delay(50); 
}

// --- HELPER FUNCTIONS ---

int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2;
}

void moveForward() {
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  analogWrite(MOTOR_A_EN, CRUISE_SPEED); // Use normal cruise speed

  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
  analogWrite(MOTOR_B_EN, CRUISE_SPEED); // Use normal cruise speed
}

void rotateRobot() {
  // Tank Turn (Motors spin in opposite directions)
  
  // Motor A Forward
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  analogWrite(MOTOR_A_EN, TURN_SPEED); // Use FASTER turn speed

  // Motor B Backward
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, HIGH);
  analogWrite(MOTOR_B_EN, TURN_SPEED); // Use FASTER turn speed
}

void stopRobot() {
  analogWrite(MOTOR_A_EN, 0);
  analogWrite(MOTOR_B_EN, 0);
}