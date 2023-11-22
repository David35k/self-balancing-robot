#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

MPU6050 mpu;

//accel stuff
int16_t accY, accZ;
float accAngle;

//gyro stuff
int16_t gyroX, gyroRate;
float gyroAngle = 0;
unsigned long currTime, prevTime = 0, loopTime;

float prevAngle = 0, currentAngle;

//PID stuff
float targetAngle = -5;
float errorSum = 0, prevError, prevIntegral;
float kP = 15;  // proportional
float kI = 0.01;   // integral
float kD = 0.5;   // derivative

//right motor
int motorENB = 5;
int motorIn3 = 4;
int motorIn4 = 9;

//left motor
int motorENA = 6;
int motorIn1 = 8;
int motorIn2 = 7;

void setup() {
  mpu.initialize();
  Serial.begin(9600);

  //right motor
  pinMode(motorENB, OUTPUT);  //pwm
  pinMode(motorIn3, OUTPUT);
  pinMode(motorIn4, OUTPUT);

  //left motor
  pinMode(motorENA, OUTPUT);  //pwm
  pinMode(motorIn2, OUTPUT);
  pinMode(motorIn1, OUTPUT);
}

void loop() {
  //get angle using accelerometer
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();

  accAngle = atan2(accY, accZ) * RAD_TO_DEG;

  // get angle using gyro
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;

  gyroX = mpu.getRotationX();
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = prevAngle + (float)gyroRate * loopTime / 1000;

  // complementary filter
  currentAngle = 0.9934 * gyroAngle + 0.0066 * accAngle;

  //PID
  float error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);
  float i = errorSum * loopTime;
  float d = (currentAngle - prevAngle) / loopTime;
  float output = kP * error + kI * i + kD * d;

  prevError = error;
  prevIntegral = i;
  prevAngle = currentAngle;


  // write to the motors
  analogWrite(motorENB, constrain(abs(output), 0, 255));
  analogWrite(motorENA, constrain(abs(output), 0, 255));

  if (output > 0) {
    //right motor
    digitalWrite(motorIn3, HIGH);
    digitalWrite(motorIn4, LOW);

    //left motor
    digitalWrite(motorIn2, LOW);
    digitalWrite(motorIn1, HIGH);
  } else {
    //right motor
    digitalWrite(motorIn3, LOW);
    digitalWrite(motorIn4, HIGH);

    //left motor
    digitalWrite(motorIn2, HIGH);
    digitalWrite(motorIn1, LOW);
  }

  Serial.print("gyroAngle:");
  Serial.println(gyroAngle);
  Serial.print("accelAngle:");
  Serial.println(accAngle);
  Serial.print("loopTime:");
  Serial.println(loopTime);
  Serial.print("target:");
  Serial.println(0);
  Serial.print("currentAngle:");
  Serial.println(currentAngle);
  Serial.print("output:");
  Serial.println(output);
  Serial.print("top:");
  Serial.println(90);
  Serial.print("bottom:");
  Serial.println(-90);
}
