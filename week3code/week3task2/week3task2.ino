#include <Wire.h>
#include <LiquidCrystal.h>  // --- ADDED ---

// --- ADDED - Motor Control Pins ---
// Define the pins for your motor driver (e.g., L298N)
// --- MODIFIED - Using your new pinout ---
#define MOTOR_A_EN 11   // Enable pin for Motor A (must be PWM)
#define MOTOR_A_IN1 13  // Direction pin 1 for Motor A
#define MOTOR_A_IN2 12  // Direction pin 2 for Motor A
#define MOTOR_B_EN 3    // Enable pin for Motor B (must be PWM)
#define MOTOR_B_IN1 2   // Direction pin 1 for Motor B (was in3 in your code)
#define MOTOR_B_IN2 1   // Direction pin 2 for Motor B (was in4 in your code)

// --- ADDED - LCD Pin Setup ---
// initialize the library with the numbers of the interface pins
// LCD(RS, enable, d4, d5, d6, d7)
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
double Kp = 20;   // Proportional gain
double Ki = 0;    // Integral gain
double Kd = 0;  // Derivative gain

double yaw_setpoint = 0;  // We want to go straight, so the target yaw is 0
double pid_error = 0;
double pid_integral = 0;
double pid_derivative = 0;
double pid_previous_error = 0;
double pid_output = 0;
int pitchState = 0;

// --- ADDED - Motor Speed Variables ---
int baseSpeed = 255;  // Base speed for both motors (0-255). Adjust this!
int motorSpeedA = 0;
int motorSpeedB = 0;
// --- END ADDED ---

const int MPU = 0x68;  // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw = 0.0;  // Initialize yaw to 0
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
unsigned long lastLCDUpdate = 0;  // --- ADDED - For non-blocking LCD updates
bool stopOperation = false;
unsigned long lastMillis = 0;
int c = 0;

void setup() {
  Wire.begin();                 // Initialize comunication
  Wire.beginTransmission(MPU);  // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);             // Talk to the register 6B
  Wire.write(0x00);             // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);   //end the transmission

  // --- ADDED - LCD Setup ---
  lcd.begin(16, 2);  // set up the LCD's number of columns and rows
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Yaw:");
  lcd.setCursor(0, 1);
  lcd.print("PID:");
  // --- END ADDED ---

  // --- ADDED - Motor Pin Setup ---
  pinMode(MOTOR_A_EN, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_EN, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  // Stop motors initially
  setMotorSpeed('A', 0);
  setMotorSpeed('B', 0);
  // --- END ADDED ---

  // Call this function to get the IMU error values for your module
  calculate_IMU_error();
  delay(20);
}

void loop() {
  previousTime = currentTime;                           // Previous time is stored before the actual time read
  currentTime = millis();                               // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000.0;  // Divide by 1000 to get seconds

  Wire.beginTransmission(MPU);
  Wire.write(0x43);  // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);                    // Read 6 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;  // deg/s
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;  // deg/s
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;  // deg/s

  // --- MODIFIED - Correct the gyro outputs with the calculated error values ---
  // We subtract the error offset
  GyroX = GyroX - GyroErrorX;  // GyroErrorX ~(-0.56)
  GyroY = GyroY - GyroErrorY;
  yaw = yaw + GyroX * elapsedTime;
  pitch = pitch + GyroY * elapsedTime;

  pid_error = yaw_setpoint - yaw;  // Calculate the error

  pid_integral = pid_integral + (pid_error * elapsedTime);
  pid_integral = constrain(pid_integral, -100, 100);

  pid_derivative = (pid_error - pid_previous_error) / elapsedTime;

  pid_output = (Kp * pid_error) + (Ki * pid_integral) + (Kd * pid_derivative);

  pid_previous_error = pid_error;

  if (!stopOperation) {
    motorSpeedB = baseSpeed + pid_output;  // Right Motor
    motorSpeedA = baseSpeed - pid_output;  // Left Motor

    // Constrain the motor speeds to be within -255 and 255
    motorSpeedA = constrain(motorSpeedA, -255, 255);
    motorSpeedB = constrain(motorSpeedB, -255, 255);

    setMotorSpeed('A', motorSpeedA);
    setMotorSpeed('B', motorSpeedB);
  } else {
    setMotorSpeed('A', 0);
    setMotorSpeed('B', 0);
  }

  if (pitchState == 0) {
    if (pitch >= 20) {
      pitchState = 2;
      delay(100);
      lastMillis = millis();
    }
  } else if (pitchState == 1) {
    if (millis() - lastMillis >= 5000) {
      stopOperation = false;
      pitchState = 2;
    }
  } else if (pitchState == 2) {
    if (pitch <= 5) {
      pitchState = 3;
      delay(200);
      stopOperation = true;
      lastMillis = millis();
    }
  } else if (pitchState == 3) {
    if (millis() - lastMillis >= 4000) {
      stopOperation = false;
      pitchState = 4;
      yaw_setpoint = 360;
    }
  } else if (pitchState == 4) {
    if (abs(pid_error) <= 25) {
      setMotorSpeed('A', 0);
      setMotorSpeed('B', 0);
      stopOperation = true;
      pitchState = 5;
      lastMillis = millis();
      pitch = 0;
    }
  }else if (pitchState == 5) {
    if (millis() - lastMillis >= 1000) {
      stopOperation = false;
      pitchState = 6;
    }
  }else if (pitchState == 6) {
    if (pitch <= -20) {
      pitchState = 7;
    }
  }else if (pitchState == 7) {
    if (pitch >= -5) {
      pitchState = 8;
      stopOperation = true;
    }
  }



  // --- END ADDED ---

  // --- ADDED - Serial Debugging ---
  // Serial.print("Yaw: ");
  // Serial.print(yaw);
  // Serial.print(" | PID_Error: ");
  // Serial.print(pid_error);
  // Serial.print(" | PID_Output: ");
  // Serial.print(pid_output);
  // Serial.print(" | SpeedA: ");
  // Serial.print(motorSpeedA);
  // Serial.print(" | SpeedB: ");
  // Serial.println(motorSpeedB);
  // --- END ADDED ---

  if (currentTime - lastLCDUpdate > 100) {
    char lcdBuffer[17];
    lcd.setCursor(0, 0);  // Position cursor after "Yaw: "
    lcd.print(yaw);
    lcd.setCursor(8, 0);
    lcd.print(pitch);
    lcd.setCursor(0, 1);  // Position cursor after "PID: "
    lcd.print(pid_error);

    lastLCDUpdate = currentTime;
  }

  // --- END ADDED ---
}

// --- ADDED - Motor Control Function ---
// This function controls a single motor.
// 'motor' is 'A' or 'B'
// 'speed' is from -255 (full reverse) to 255 (full forward)
void setMotorSpeed(char motor, int speed) {
  int enPin, in1Pin, in2Pin;

  if (motor == 'A') {
    enPin = MOTOR_A_EN;
    in1Pin = MOTOR_A_IN1;
    in2Pin = MOTOR_A_IN2;
  } else if (motor == 'B') {
    enPin = MOTOR_B_EN;
    in1Pin = MOTOR_B_IN1;
    in2Pin = MOTOR_B_IN2;
  } else {
    return;  // Invalid motor
  }

  if (speed > 0) {
    // Go forward
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(enPin, constrain(abs(speed), 100, 255));
  } else if (speed < 0) {
    // Go backward
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(enPin, constrain(abs(speed), 100, 255));
  } else {
    // Stop
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(enPin, 0);
  }
}
// --- END ADDED ---

void calculate_IMU_error() {
  // This function is unchanged from your original code.
  // It calculates the gyro offsets (GyroErrorX, Y, Z)
  c = 0;
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
    // These are calculating pitch and roll, not used for yaw but good to have
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }

  // Calculate the average gyro error
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;

  // Print the errors
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}