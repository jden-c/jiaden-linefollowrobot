#include <LiquidCrystal.h>

const bool INVERT_A = false; 
const bool INVERT_B = false;

// --- PINS ---
#define enA 3    
#define in1 1  
#define in2 2    

#define enB 11   
#define in3 12   
#define in4 13   

const int encoderRightPin = A1;
const int encoderLeftPin = A2;
const int midIrPin = A3;
const int leftIrPin = A4;
const int rightIrPin = A5;

float Kp = 10.0;  // Increased Kp for snappier reaction
float Ki = 0.0;   
float Kd = 6.0;  

#define LEFT_THRES 550
#define RIGHT_THRES 200
#define MID_THRES 500

#define FORWARD 1
#define BACKWARD 0
#define CM_PER_COUNT 0.5

#define STATE_RUNNING_TO_20 0
#define STATE_PAUSED 1
#define STATE_RUNNING_TO_END 2
#define STATE_FINISHED 3

#define BASE_SPEED 150   
#define MAX_SPEED 220    
#define TURN_SPEED 200  // Speed for the "Tank Turn"

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

volatile unsigned long rightCount = 0;
volatile unsigned long leftCount = 0;

int currentState = STATE_RUNNING_TO_20;
float lastError = 0;
int lastDetectedDirection = 1; // 1 = Right, -1 = Left (Start with assumption)

int last = 0;
int turnSpeed = 150;
int straightSpeed = 150;

unsigned long totalRunTime = 0;
unsigned long lastLoopTime = 0;
unsigned long pauseStartTime = 0;

int stopCounter = 0;
int Counter = 0;

void rotateMotorA(int speed, int direction) {
  speed = constrain(speed, 0, 255);
  analogWrite(enA, speed);
  if (direction == FORWARD) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
}

void rotateMotorB(int speed, int direction) {
  speed = constrain(speed, 0, 255);
  analogWrite(enB, speed);
  if (direction == FORWARD) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void displayStats(unsigned long sec, unsigned long ms, float dist) {

  char formattedTime[8];
  sprintf(formattedTime, "%02ld:%03ld", sec, ms);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Time: ");
  lcd.setCursor(6, 0);
  lcd.print(formattedTime);

  lcd.setCursor(0, 1);
  lcd.print("Dist: ");
  lcd.print(dist);
  lcd.print(" cm");
}


void lineFollowLogic(int leftVal, int midVal, int rightVal) {
  int threshold = 500;

  bool leftOnLine = leftVal < threshold;
  bool midOnLine = midVal > threshold;
  bool rightOnLine = rightVal < threshold;

  // lcd.setCursor(12, 1);
  // lcd.print(leftOnLine);

  // lcd.setCursor(13, 1);
  // lcd.print(midOnLine);

  // lcd.setCursor(14, 1);
  // lcd.print(rightOnLine);
  // Is the robot completely lost? (All White)
  bool allSensorsWhite = (!leftOnLine && !midOnLine && !rightOnLine);

  int leftSpeed = 0;
  int rightSpeed = 0;
  if (allSensorsWhite) {
    // --- RECOVERY MODE ---
    // We rely on 'lastDetectedDirection' which we saved earlier.

    if (lastDetectedDirection == -1) {
      // Line was on LEFT last -> Hard Turn LEFT
      leftSpeed = -TURN_SPEED;
      rightSpeed = TURN_SPEED;
    } else {
      // Line was on RIGHT last -> Hard Turn RIGHT
      leftSpeed = TURN_SPEED;
      rightSpeed = -TURN_SPEED;
    }
    // stopMotors();
    // while(1);

  } else {
    // --- NORMAL PID MODE ---

    int error = 0;

    if (midOnLine && !leftOnLine && !rightOnLine) {
      error = 0;
    } else {
      long totalValue = leftVal + midVal + rightVal;
      if (totalValue == 0) totalValue = 1;
      long weightedSum = ((long)leftVal * 1000) - ((long)rightVal * 1000);
      error = weightedSum / totalValue;
    }

    // --- UPDATE STICKY DIRECTION ---
    // Only update if the error is significant.
    // If error is > 0, line is on Left. If < 0, line is on Right.
    if (error > 200) {
      lastDetectedDirection = -1;  // Remember Left
    } else if (error < -200) {
      lastDetectedDirection = 1;  // Remember Right
    }

    // PID
    float P = error;
    float D = error - lastError;
    lastError = error;

    int motorCorrection = (Kp * P) + (Kd * D);

    leftSpeed = BASE_SPEED + motorCorrection;
    rightSpeed = BASE_SPEED - motorCorrection;

    if (leftSpeed > MAX_SPEED) leftSpeed = MAX_SPEED;
    if (rightSpeed > MAX_SPEED) rightSpeed = MAX_SPEED;

    // In Normal Mode, do not reverse (keeps it smooth)
    if (leftSpeed < 0) leftSpeed = 0;
    if (rightSpeed < 0) rightSpeed = 0;
  }

  driveMotorA(leftSpeed);
  driveMotorB(rightSpeed);
}


void driveMotorA(int speed) {
    bool forward = (speed >= 0);
    if (INVERT_A) forward = !forward;

    if (forward) {
        analogWrite(enA, abs(speed));
        digitalWrite(in1, LOW); 
        digitalWrite(in2, HIGH);
    } else {
        analogWrite(enA, abs(speed)); 
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }
}

void driveMotorB(int speed) {
    bool forward = (speed >= 0);
    if (INVERT_B) forward = !forward;

    if (forward) {
        analogWrite(enB, abs(speed));
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
    } else {
        analogWrite(enB, abs(speed));
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
    }
}

ISR(PCINT1_vect) {
  uint8_t portState = PINC;
  int rightState = (portState >> 1) & 1;
  int leftState = (portState >> 2) & 1;
  static int lastRightState = 0;
  static int lastLeftState = 0;
  if (rightState != lastRightState) rightCount++;
  if (leftState != lastLeftState) leftCount++;
  lastRightState = rightState;
  lastLeftState = leftState;
}

void setup() {
  lcd.begin(16, 2);
  lcd.clear();

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(rightIrPin, INPUT);
  pinMode(leftIrPin, INPUT);
  pinMode(midIrPin, INPUT);
  pinMode(encoderRightPin, INPUT);
  pinMode(encoderLeftPin, INPUT);

  PCICR |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT9) | (1 << PCINT10);

  stopMotors();
  lastLoopTime = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  float avgCounts = (leftCount + rightCount) / 2.0;
  float distance = avgCounts * CM_PER_COUNT;

  int leftVal = analogRead(leftIrPin);
  int midVal = analogRead(midIrPin);
  int rightVal = analogRead(rightIrPin);


  if (currentState != STATE_FINISHED) {
    totalRunTime += (currentMillis - lastLoopTime);
  }
  lastLoopTime = currentMillis;

  unsigned long seconds = totalRunTime / 1000;
  unsigned long remainder = totalRunTime % 1000;

  switch (currentState) {

    case STATE_RUNNING_TO_20:
      lineFollowLogic(leftVal, midVal, rightVal);
      displayStats(seconds, remainder, distance);

      if (distance >= 366.0) {
        stopMotors();
        pauseStartTime = currentMillis;
        currentState = STATE_PAUSED;
        lcd.clear();
      }
      break;

    case STATE_PAUSED:
      stopMotors();
      distance = 20.09;
      unsigned long timeInPause;
      timeInPause = currentMillis - pauseStartTime;

      if (timeInPause >= 3000) {
        rotateMotorA(straightSpeed, BACKWARD);
        rotateMotorB(straightSpeed, BACKWARD);
        delay(50);
        currentState = STATE_RUNNING_TO_END;
        lcd.clear();
      } else {
        int remainingSeconds = 3 - timeInPause / 1000;
        lcd.setCursor(0, 0);
        lcd.print("Wait: ");
        lcd.print(remainingSeconds);
        lcd.print(" s");

        lcd.setCursor(0, 1);
        lcd.print("Dist: ");
        lcd.print(distance);
      }
      break;

    case STATE_RUNNING_TO_END:

      lineFollowLogic(leftVal, midVal, rightVal);
      displayStats(seconds, remainder, distance);

      if (leftVal < LEFT_THRES && rightVal < RIGHT_THRES && midVal > MID_THRES) {
        stopCounter++;
        if (stopCounter > 1) {
          stopMotors();
          currentState = STATE_FINISHED;
          lcd.clear();
        }
      } else {
        stopCounter = 0;
      }
      break;

    case STATE_FINISHED:
      stopMotors();
      char formattedTime[8];
      sprintf(formattedTime, "%02ld:%03ld", seconds, remainder);

      lcd.setCursor(0, 0);
      lcd.print("Time: ");
      lcd.setCursor(6, 0);
      lcd.print(formattedTime);

      lcd.setCursor(0, 1);
      lcd.print("Dist: ");
      lcd.print(distance);
      lcd.print(" cm");
      break;
  }
}