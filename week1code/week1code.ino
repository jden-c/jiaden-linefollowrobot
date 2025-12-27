#include <LiquidCrystal.h>

#define enA 11  
#define in1 13  
#define in2 12  

#define enB 3  
#define in3 2  
#define in4 1  

#define FORWARD 1
#define BACKWARD 0

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
unsigned long lastMillis;
unsigned long lastLCD;

void rotateMotorA(int speed, int direction) {
  analogWrite(enA, speed);  
  if (direction == FORWARD) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else if (direction == BACKWARD) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
}

void rotateMotorB(int speed, int direction) {
  analogWrite(enB, speed);  
  if (direction == FORWARD) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else if (direction == BACKWARD) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
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

void setup() {
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Time");

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  stopMotors();
  lastMillis = millis();
}

void loop() {
  lcd.setCursor(0, 1);
  unsigned long milliseconds = (millis() - lastMillis) % 1000;
  unsigned long seconds = (millis() - lastMillis) / 1000;
  char formattedTime[7];
  sprintf(formattedTime, "%02ld:%03ld", seconds, milliseconds);
  lcd.print(formattedTime);
  if (millis() - lastMillis < 10000) {
    rotateMotorA(255, FORWARD);
    rotateMotorB(253, FORWARD);
  } else {
    stopMotors();
    while (1)
      ;
  }
}