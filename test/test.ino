#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>

#define PI 3.14159

Adafruit_MPU6050 mpu;

float xn1 = 0;
float yn1 = 0;

//right motor
int motorENB = 5;
int motorIn3 = 4;
int motorIn4 = 9;

//left motor
int motorENA = 6;
int motorIn1 = 8;
int motorIn2 = 7;

float oldAngleGyro = 6969;

// PID stuff
float targetAngle = 90;
float prevError = 0;
float prevIntegral = 0;
float kP = 0.2;
float kI = 0;
float kD = 0;
// float kP = 0.1;
// float kI = 0.03;
// float kD = 0.005;
float bias = 0;
float iterationTime = 0.02;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // try to inialize the mpu6050
  if (!mpu.begin()) {
    Serial.println("cant find mpu6050 XD!!");
    while (1) {
      delay(10);
    }
  }

  Serial.println("yessir, found the mpu6050");

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  //right motor
  pinMode(motorENB, OUTPUT);  //pwm
  pinMode(motorIn3, OUTPUT);
  pinMode(motorIn4, OUTPUT);

  //left motor
  pinMode(motorENA, OUTPUT);  //pwm
  pinMode(motorIn2, OUTPUT);
  pinMode(motorIn1, OUTPUT);

  delay(100);
}

void loop() {
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  // ----- degrees using accel -----

  float ratio = constrain(constrain(a.acceleration.z, 0.01, 9.5) / constrain(a.acceleration.y, -9.5, 9.5), -6, 6);

  float eq = 72.5003 * pow(abs(ratio), 0.253558) - 22.8643;
  float angleAccel;
  if (ratio > 0) {
    angleAccel = 90 + (90 - eq);
  } else if (ratio < 0) {
    angleAccel = eq;
  } else if (ratio == 0) {
    angleAccel = 90;
  }

  //sus stuff
  if(oldAngleGyro == 6969) {
    oldAngleGyro = angleAccel;
  }

  

  // ----- degrees using gyro -----

  float newAngleGyro = oldAngleGyro + (g.gyro.x * 180 / PI) * iterationTime;

  if (newAngleGyro - angleAccel > 10 || newAngleGyro - angleAccel < -10) {
    newAngleGyro = angleAccel;
  }

  oldAngleGyro = newAngleGyro;

  //complementary filter - idk if im doing this right :skull:
  float compAngle = (0.5 * newAngleGyro) + (0.5 * angleAccel);

  //PID
  float error = targetAngle - compAngle;
  float i = prevIntegral + error * iterationTime;
  float d = (error - prevError) / iterationTime;
  float output = kP * error + kI * i + kD * d;

  prevError = error;
  prevIntegral = i;

   if (outputLowPass < 0) {
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

  int sus = constrain(abs(outputLowPass), 0, 10);
  sus = map(sus, 0, 10, 50, 255);

  //low pass filter - makes it nice and smooth
  float filt = 0.5;
  float susLowPass = (1 - filt) * yn1 + (filt / 2) * xn1 + (filt / 2) * sus;

  xn1 = sus;
  yn1 = susLowPass;

  analogWrite(motorENB, susLowPass);
  analogWrite(motorENA, susLowPass);
  // analogWrite(motorENA, constrain(abs(output*5), 55, 175));
  // analogWrite(motorENB, constrain(abs(output), 150, 250));
  // analogWrite(motorENA, constrain(abs(output), 150, 175));

 

  Serial.print("angle_gyro:");
  Serial.print(newAngleGyro);
  Serial.print(",");
  // Serial.print("angle_accel_low_pass:");
  // Serial.print(angleAccelLowPass);
  Serial.print(",");
  Serial.print("angle_accel:");
  Serial.print(angleAccel);
  Serial.print(",");
  Serial.print("complementary_angle:");
  Serial.print(compAngle);
  Serial.print(",");
  Serial.print("target:");
  Serial.println(90);
  Serial.print(",");
  Serial.print("outputLowPass:");
  Serial.println(outputLowPass);
  Serial.print(",");
  Serial.print("motors:");
  Serial.println(map(susLowPass, 0, 10, 50, 255));

  Serial.println("");
  delay(iterationTime * 1000);
}
