/*
  Restaurant Line Following Code for F-Dash
*/

// motor pins
#include <SparkFun_TB6612.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 10
#define PWMA 5
#define PWMB 6


// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;


Motor motor1 = Motor(IN1, IN2, PWMA, offsetA, PWMA); // right motors
Motor motor2 = Motor(IN3, IN4, PWMB, offsetB, PWMB); // left motors

// SONAR pins
#define trig 14
#define echo 15
#define MAX_DISTANCE 10

// buzzer pin
#define buzzer 4

// defining OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


// program variables
int duration, distance;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 140;

float Kp = 0;
float Kd = 0;
float Ki = 0 ;

int minValues[6], maxValues[6], threshold[6];

void setup()
{
  Serial.begin(115200);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(buzzer, OUTPUT);


  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(40, 10);
  // Display static text
  display.println("F-DASH");
  display.setCursor(40, 20);
  // Display static text
  display.println("eat fast");
  display.display();
}


void loop()
{
  while (digitalRead(11) == LOW) {}
    delay(1000);
    calibrate();
    while (digitalRead(12) == LOW ) {}
    delay(1000);

    while (1)
    {
    if (analogRead(1) > threshold[1] && analogRead(5) < threshold[5] )  // make a left turn
    {
      lsp = 0; rsp = lfspeed;
      motor1.drive(lfspeed);
      motor2.drive(0);
    }

    else if (analogRead(5) > threshold[5] && analogRead(1) < threshold[1])  // make a right turn
    { lsp = lfspeed; rsp = 0;
      motor1.drive(0);
      motor2.drive(lfspeed);
    }
    else if (analogRead(3) > threshold[3])
    {
      Kp = 0.0006 * (1000 - analogRead(3)); //kp = 0.0006 // line following block
      Kd = 10 * Kp;
      //Ki = 0.0001;
      //linefollow();
      FDASH_Delivery();
    }
    }
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
  motor1.drive(rsp);
  motor2.drive(lsp);

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
    motor1.drive(-120);
    motor2.drive(120);

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

  motor1.drive(0);
  motor2.drive(0);
}

int get_distance() {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(2);
  digitalWrite(trig, LOW);
  delayMicroseconds(10);

  duration = pulseIn(echo, HIGH);
  distance = (duration * 0.034) / 2;

  return distance;
}

void FDASH_Delivery() {
  // define the limits of operations
  if (distance < 2 || distance > MAX_DISTANCE) distance = MAX_DISTANCE;
  if (distance > 10) {
    // start delivery process
    linefollow();
  }

  else {
    // stop delivery process
    tone(buzzer, 2000);
  }
}

void Display(){
  
}
