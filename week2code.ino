#include <LiquidCrystal.h>

// --- Motor Pins ---
#define enA 3   
#define in1 1  
#define in2 2   
#define enB 11  
#define in3 12  
#define in4 13  

// --- Sensor Pins ---
const int lineFollowRightPin = A5;
const int lineFollowLeftPin = A4;
const int encoderRightPin = A1;
const int encoderLeftPin = A2;

// --- Tuning Constants ---
float Kp = 6.0;   
float Ki = 0.0;   
float Kd = 3.0;   

// --- Speed Settings ---
#define BASE_SPEED 100   
#define MAX_SPEED 200    

// --- 90 Degree Turn Settings (NEW) ---
// You must tune this value! Start with 20, increase until it turns exactly 90 deg.
#define TURN_TARGET_COUNTS 25  
// Threshold to decide if a sensor sees the line (Adjust based on your environment)
#define LINE_THRESHOLD 600     

// --- Global Variables ---
float lastError = 0;
volatile unsigned long rightCount = 0;
volatile unsigned long leftCount = 0;
unsigned long lastMillis;
unsigned long lastTurnTime = 0; // Debounce for turns

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// ------------------------------------------------------------------

void setup() {
    lcd.begin(16, 2);
    lcd.clear();
    lcd.print("Ready to Run");

    pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);

    pinMode(lineFollowRightPin, INPUT);
    pinMode(lineFollowLeftPin, INPUT);
    pinMode(encoderRightPin, INPUT);
    pinMode(encoderLeftPin, INPUT);

    // Interrupts for Encoders
    PCICR |= (1 << PCIE1);     
    PCMSK1 |= (1 << PCINT9) | (1 << PCINT10); 

    stopMotors();
    delay(1000); 
    lastMillis = millis();
}

void loop() {
    int leftVal = analogRead(lineFollowLeftPin);
    int rightVal = analogRead(lineFollowRightPin);

    // --- 1. DETECT 90 DEGREE INTERSECTION ---
    // If BOTH sensors see the line (High Value > Threshold)
    // We assume it's a T-Junction or 90-degree marker.
    // We add a timer check (lastTurnTime) so it doesn't trigger twice in a row.
    if (leftVal > LINE_THRESHOLD && rightVal > LINE_THRESHOLD && (millis() - lastTurnTime > 1000)) {
        
        stopMotors();
        lcd.setCursor(0,0);
        lcd.print("90 Deg Turn");
        
        // Example: Turn RIGHT 90 degrees. 
        // Change to 'false' if you want to turn LEFT.
        turn90(true); 
        
        lastTurnTime = millis(); // Reset timer
        lastError = 0; // Reset PID memory
        return; // Skip the rest of the loop to resume fresh
    }

    // --- 2. STANDARD PID CONTROL ---
    int error = leftVal - rightVal;

    // Centered logic (White line on Black floor logic)
    if (leftVal > LINE_THRESHOLD && rightVal > LINE_THRESHOLD && abs(error) < 100) {
        error = 0;
    }

    float P = error;
    float D = error - lastError;
    lastError = error;

    int motorCorrection = (Kp * P) + (Kd * D);

    int leftSpeed = BASE_SPEED - motorCorrection;
    int rightSpeed = BASE_SPEED + motorCorrection;

    if (leftSpeed > MAX_SPEED) leftSpeed = MAX_SPEED;
    if (rightSpeed > MAX_SPEED) rightSpeed = MAX_SPEED;
    if (leftSpeed < -100) leftSpeed = -100;
    if (rightSpeed < -100) rightSpeed = -100;

    driveMotorA(leftSpeed);
    driveMotorB(rightSpeed);
}

// --- NEW FUNCTION: Precise 90 Degree Turn ---
void turn90(bool turnRight) {
    // 1. Reset Encoders
    rightCount = 0;
    leftCount = 0;

    // 2. Start Spinning
    // A speed of 120-150 is usually good for pivoting
    int turnSpeed = 130; 

    if (turnRight) {
        // Tank Turn Right: Left Forward, Right Backward
        driveMotorA(turnSpeed);  
        driveMotorB(-turnSpeed); 
    } else {
        // Tank Turn Left: Left Backward, Right Forward
        driveMotorA(-turnSpeed); 
        driveMotorB(turnSpeed);  
    }

    // 3. Wait until distance reached
    // We average the counts or just check one side. 
    // Checking both ensures we have actually moved.
    while((rightCount + leftCount) / 2 < TURN_TARGET_COUNTS) {
        // Use this loop to safeguard against getting stuck
        if (millis() - lastTurnTime > 2000) break; // Timeout after 2 sec
    }

    // 4. Stop
    stopMotors();
    delay(200); // Brief pause to stabilize
}

// --- Motor & ISR Functions (Unchanged) ---
void driveMotorA(int speed) {
    if (speed >= 0) {
        analogWrite(enA, speed); digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    } else {
        analogWrite(enA, -speed); digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    }
}

void driveMotorB(int speed) {
    if (speed >= 0) {
        analogWrite(enB, speed); digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
    } else {
        analogWrite(enB, -speed); digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
    }
}

void stopMotors() {
    analogWrite(enA, 0); analogWrite(enB, 0);
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

ISR(PCINT1_vect) {
    uint8_t portState = PINC;
    static int lastRightState = 0;
    static int lastLeftState = 0;
    int rightState = (portState >> 1) & 1; 
    int leftState = (portState >> 2) & 1;  

    if (rightState != lastRightState) rightCount++;
    if (leftState != lastLeftState) leftCount++;

    lastRightState = rightState;
    lastLeftState = leftState;
}