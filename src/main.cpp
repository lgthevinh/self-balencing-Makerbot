#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <KalmanFilter.h>
#include <Wire.h>

#define rightMotor_phaseA 25
#define rightMotor_phaseB 32
#define leftMotor_phaseA 36
#define leftMotor_phaseB 39

int pulse_right = 0;
int pulse_left = 0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_MPU6050 mpu = Adafruit_MPU6050();

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

//PID CONSTANST (Experimental)
float elapsedTime, time_run, timePrev;
float kp = 10;
float ki = 0.3;
float kd = 50;
float targetAngle = 0;

/*******************/

//Initialise MPU variables
float accPitch = 0;
float kalPitch = 0;

//Declare Global Variables
float PID, error, previousError, Speed;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;

float speed;

void right_(){
  short input = digitalRead(rightMotor_phaseB);
  if(input == 1) pulse_right++;
  if(input == 0) pulse_right--;
}

void left_(){
  short input = digitalRead(leftMotor_phaseB);
  if(input == 1) pulse_left++;
  if(input == 0) pulse_left--;
}

void setMotorSpeed(int left, int right, bool dir = 0){
  if(dir == 0){
    pwm.setPWM(8, 0, left);
    pwm.setPWM(9, 0, 0);
    pwm.setPWM(11, 0, right);
    pwm.setPWM(10, 0, 0);
  } else {
    pwm.setPWM(8, 0, 0);
    pwm.setPWM(9, 0, left);
    pwm.setPWM(11, 0, 0);
    pwm.setPWM(10, 0, right);
  }
}

void setup() {
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(1000);
  Serial.begin(115200);

  mpu.begin(0x69);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);

  pinMode(rightMotor_phaseA, INPUT_PULLUP);
  pinMode(rightMotor_phaseB, INPUT_PULLUP);
  pinMode(leftMotor_phaseA, INPUT_PULLUP);
  pinMode(leftMotor_phaseB, INPUT_PULLUP);
  attachInterrupt(rightMotor_phaseA, right_, FALLING);
  attachInterrupt(leftMotor_phaseA, left_, FALLING);

  time_run = millis();
}

void loop(){
  timePrev = time_run;
  time_run = millis();
  elapsedTime = (time_run - timePrev) / 1000;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  accPitch = -(atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0) / M_PI;
  
  //Kalman filter
  kalPitch = kalmanY.update(accPitch, g.gyro.y);

  //Current Error in Angle
  error = kalPitch - targetAngle;

  //Proportional Error
  pid_p = kp * error;

  //Integral Error
  pid_i = (ki * error);
  //Differential Error
  pid_d = kd * ((error - previousError) / elapsedTime);

  //Total PID Value
  PID = pid_p + pid_i + pid_d;

  speed = abs(PID);

  previousError = error;


  if (kalPitch > 0) setMotorSpeed(speed, speed, 1);
  if (kalPitch < 0) setMotorSpeed(speed, speed);
  if (kalPitch > 45 || kalPitch < -45) setMotorSpeed(0, 0);
  
  if (kalPitch > 45 || kalPitch < -45) setMotorSpeed(4060, 4060);
 
 Serial.println(speed);
}
