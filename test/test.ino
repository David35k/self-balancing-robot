#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>

#define PI 3.14159

Adafruit_MPU6050 mpu;

//right motor
int motorENB = 5;
int motorIn3 = 4;
int motorIn4 = 9;

//left motor
int motorENA = 6;
int motorIn1 = 8;
int motorIn2 = 7;

float oldAngleGyro = 90;


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
  // put your main code here, to run repeatedly:
  // analogWrite(motorENB, 255);
  digitalWrite(motorIn3, HIGH);
  digitalWrite(motorIn4, LOW);

  // analogWrite(motorENA, 175);
  digitalWrite(motorIn2, LOW);
  digitalWrite(motorIn1, HIGH);


  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  //degrees using accel

  float ratio = constrain(constrain(a.acceleration.z, 0.01, 9.5) / constrain(a.acceleration.x, -9.5, 9.5), -6, 6);

  float eq = 72.5003 * pow(abs(ratio), 0.253558) - 22.8643;
  float angleAccel;
  if (ratio < 0) {
    angleAccel = 90 + (90 - eq);
  } else if (ratio > 0) {
    angleAccel = eq;
  }

  //degrees using gyro

  float newAngleGyro = oldAngleGyro + (g.gyro.y * 180 / PI) * 0.05;

  float compAngle = (0.1 * newAngleGyro)+ (0.9 * angleAccel);

  Serial.print("angle_gyro:");
  Serial.print(newAngleGyro);
  Serial.print(",");
  Serial.print("angle_accel:");
  Serial.print(angleAccel);
  Serial.print(",");
  Serial.print("angle_accel_lowpass:");
  Serial.print(angleAccelLowPass);
  Serial.print(",");
  Serial.print("complementary_angle:");
  Serial.print(compAngle);
  Serial.print(",");
  Serial.print("static_var:");
  Serial.println(90);

  oldAngleGyro = newAngleGyro;

  Serial.println("");
  delay(50);
}
