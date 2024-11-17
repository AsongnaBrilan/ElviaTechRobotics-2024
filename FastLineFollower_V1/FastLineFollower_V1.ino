/*
  Author: Asongna Brilan
  Email: niconerd650239348@gmail.com

  This is a full code for a fast line following robot designed to be used in competitions
  using Arduino Nano .
  To use this code effectively, first twick and carry individual test for each components.
*/

#include <Arduino.h>

// Motor Pins based on L293D motor driver
#define PWMA 9
#define PWMB 8
#define IN1 5
#define IN2 4
#define IN3 6
#define IN4 7

// Program variables
int MAXSPEED = 255;
int MINSPEED = 100;
int s[5];
int threshold = 512; // default
int min_threshold = 0, max_threshold = 1023;
float current_error, prev_current_error;
float avg;
char turn;
int bal = 0;

const int max_pwm = 250;
int sensor_pos = 0;

int min_readings[5];
int max_readings[5];
int sensor_readings[5];

// Setting PID constants
float kp = 0;
float ki = 0;
float kd = 0;

void setup() {
  Serial.begin(9600); // needs to uncommented when compiling final code.
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(13, OUTPUT);

  //digitalWrite(PWMA, HIGH);
  //digitalWrite(PWMB, HIGH);
  calibrate_sensors();
}

void loop() {
  PID_LINE_FOLLOW();
}

// calibrate sensors

void calibrate_sensors() {
  // init min and max values for the sensors
  for (int k = 0; k < 5 ; k++) {
    min_readings[k] = analogRead(A0 + k);
    min_readings[k] = analogRead(A0 + k);
  }

  // move the robot to calibrate sensor readings
  Drive(100, -100, 1, 0, 0, 1); // turn robot right
  delay(1000); // allow some time for calibration

  // Read sensor values multiple times to find max and  min
  for (int i = 0; i < 100; i++) {
    for (byte j = 0; j < 5; j++) {
      sensor_readings[j] = analogRead(A0 + j);
      if (sensor_readings[j] < min_readings[j]) min_readings[j] = sensor_readings[j];
      if (sensor_readings[j] > max_readings[j]) max_readings[j] = sensor_readings[j];
    }
    delay(10);
  }

  // calculate the threshold as the average of min and max values
  int sum_min = 0, sum_max = 0;
  for (int i = 0; i < 5; i++) {
    sum_min += min_readings[i];
    sum_max += max_readings[i];
  }

  min_threshold = sum_min / 5;
  max_threshold = sum_max / 5;
  threshold = (min_threshold + max_threshold) / 2;

}

// Sensor readinds
void sensor_reading() {
  sensor_pos = 0;
  int active_sensors = 0;

  for (byte i = 0; i < 5; i++) {
    s[i] = analogRead(A0 + i); // Read analog value from A0 - A4 
    if (s[i] > threshold) s[i] = 1;
    else s[i] = 0;
    if (s[i] == 1) active_sensors++;
  }

  sensor_pos = (s[0] * 1 + s[1] * 2 + s[2] * 4 + s[3] * 8 + s[4] * 16);
  if (active_sensors > 0) {
    avg = sensor_pos / active_sensors;  // Average value
  }
}

// PID Line Following
void PID_LINE_FOLLOW() {
  int P, I, D, PID;
  int left_motor, right_motor;
  int turn_speed = 170;  // Turn speed, adjust as needed

  float setpoint = 12;     // Target value for the sensor

  // Read sensors and compute the average position
  sensor_reading();

  // Calculate PID components
  current_error = setpoint - avg;
  P = current_error * kp;
  I += current_error;  // Accumulate the integral term
  D = kd * (current_error - prev_current_error);

  // Compute PID value and prevent integral windup
  PID = P + (ki * I) + D;
  I = constrain(I, -255, 255); // Prevent integral windup
  prev_current_error = current_error;

  left_motor = max_pwm + PID;
  right_motor = max_pwm - PID;

  // Ensure motor speed is within valid range
  left_motor = constrain(left_motor, -max_pwm, max_pwm);
  right_motor = constrain(right_motor, -max_pwm, max_pwm);

  // Set motor speeds
  Drive(left_motor + bal, right_motor, 1, 0, 0, 1);

  // Handle turns when sensors lose line
  if ((s[0] + s[1] + s[2] + s[3] + s[4]) == 0) {
    if (turn != 's') {
      Drive(0, 0, 1, 0, 0, 1);
      if (turn == 'r') {
        Drive(turn_speed, -turn_speed - bal, 1, 0, 0, 1);
      } else {
        Drive(-turn_speed, turn_speed + bal, 1, 0, 0, 1);
      }
      while (s[2] == 0) sensor_reading();
      turn = 's';
    }
  }

  if (s[0] == 0 && s[4] == 1) turn = 'l';
  if (s[4] == 0 && s[0] == 1) turn = 'r';

  else if ((s[0] + s[1] + s[2] + s[3] + s[4]) == 5) {
    sensor_reading();
    if ((s[0] + s[1] + s[2] + s[3] + s[4]) == 5) {
      Drive(0, 0, 1, 0, 0, 1);
      while ((s[0] + s[1] + s[2] + s[3] + s[4]) == 5) sensor_reading();
    } else if ((s[0] + s[1] + s[2] + s[3] + s[4]) == 0) turn = 'r';
  }

}

// Serial debugging

void show_analog_value() {
  for (short int i = 5; i >= 0; i--) {
    Serial.print(String(analogRead(A0 + i)) + " ");
  }
  delay(100);
}


// Motor control
void Drive(int LS, int RS, int in1, int in2, int in3, int in4) {
  /*
     Motor speed control.
     Min PWM = 100, Max PWM = 255

     Use this to set you motors
     Drive(MAXSPEED, MAXSPEED, 0, 1, 1, 0); --> motors move backward
     Drive(MAXSPEED, MAXSPEED, 1, 0, 0, 1); --> motors move forward

     For turning left, increase right motor speed or reduce left motor speed
     For turning right, increase left motor speed or reduce right motor speed
     Drive(MAXSPEED, MAXSPEED, 0, 1, 1, 0);
     Drive(MAXSPEED, MAXSPEED, 0, 1, 1, 0);
  */

  analogWrite(PWMA, RS);
  analogWrite(PWMB, LS);

  // Motor direction control
  digitalWrite(IN1, in1);
  digitalWrite(IN2, in2);
  digitalWrite(IN3, in3);
  digitalWrite(IN4, in4);
}

// System check
void Blink() {
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
}
