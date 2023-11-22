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


int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 140;

float Kp = 0;
float Kd = 0;
float Ki = 0;

//bool cal_state = HIGH;
//bool drive_state = HIGH;

int minValues[6], maxValues[6], threshold[6];

void setup()
{
  Serial.begin(9600);
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(13, OUTPUT);
  
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
}


void loop()
{
  sensorTest();
  /*while (digitalRead(11) == LOW) {}
  delay(1000);
  calibrate();
  while (digitalRead(12) == LOW ) {}
  delay(1000);

  while (1)
  {
    if (analogRead(1) > threshold[1] && analogRead(5) < threshold[5] )  // make a left turn
    {
      lsp = 0; rsp = lfspeed;
      //motor1.drive(lfspeed); Right Motor
      //motor2.drive(0); Left Motor
      Drive(0, lfspeed, 1, 0, 0, 1);
    }

    else if (analogRead(5) > threshold[5] && analogRead(1) < threshold[1])  // make a right turn
    { lsp = lfspeed; rsp = 0;
      //motor1.drive(0);
      //motor2.drive(lfspeed);
      Drive(lfspeed, 0, 1, 0, 0, 1);
    }
    else if (analogRead(3) > threshold[3])
    {
      Kp = 0.002 * (1000 - analogRead(3)); //kp = 0.0006 // line following block
      Kd = 10 * Kp;
      //Ki = 0.0001;
      linefollow();
    }
  }*/
}

void linefollow()
{
  int error = (analogRead(2) - analogRead(4));

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < 0) {
    rsp = 0;
  }
  //motor1.drive(rsp);
  //motor2.drive(lsp);
  Drive(lsp, rsp, 1, 0, 0, 1);

}

void calibrate()
{
  for ( int i = 1; i < 6; i++)
  {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }
  
  for (int i = 0; i < 3000; i++)
  {
    //motor1.drive(-120);
    //motor2.drive(120);
    Drive(120, -120, 1, 0, 0, 1);
    

    for ( int i = 1; i < 6; i++)
    {
      if (analogRead(i) < minValues[i])
      {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i])
      {
        maxValues[i] = analogRead(i);
      }
    }
  }

  for ( int i = 1; i < 6; i++)
  {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
  }
  Serial.println();
  
   Drive(0, 0, 1, 0, 0, 1);
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

void sensorTest(){
  int sensor_data[5];
  for (int i = 0; i<5; i++){
    sensor_data[i] = analogRead(A0 + i);
    Serial.print(sensor_data[i]);
    Serial.print("  ");
  }
  Serial.println();
}
