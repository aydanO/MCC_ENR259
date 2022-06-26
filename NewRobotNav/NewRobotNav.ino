/*
  ENG 259 Robot
  Authors:
  Hanning, Eligh
  Hoodak, Sam
  Johnson, Andrew
  Nguyen, Duc
  O' Brien, Aydan
  Serbetci, Suleyman
  Tretyak, Bogdan
  Professor: Berl, Geoffrey
*/

//Include libraries for motor driver and sensor array
#include <QTRSensors.h>
#include <Wire.h>
#include <WireIMXRT.h>
#include <WireKinetis.h>
#include "TB67H420FTG.h"
#include "Adafruit_TCS34725.h"
#include <Servo.h>

Adafruit_TCS34725 fishSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 pondSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


const unsigned char MAX_SPEED = 50;

// Motor Driver Properties
TB67H420FTG driver(5, 6, 9, 7, 8, 10);   //Pin numbers on Teensy

Servo DumpServo;
Servo ArmServo;
Servo HandServo;

//Setup function to calibrate line sensor array
void setup() {
  driver.init();
  Serial.begin(9600);

  DumpServo.attach(3, 490, 2690);
  ArmServo.attach(2, 900, 2450);
  HandServo.attach(4, 900, 2250);

  ServoHome();

  TwoWire *teensyWire = &Wire;
  if (pondSensor.begin(TCS34725_ADDRESS, teensyWire)) {
    //Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (true); // halt!
  }

  TwoWire *teensyWire1 = &Wire1;
  if (fishSensor.begin(TCS34725_ADDRESS, teensyWire1)) {
    //Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (true); // halt!
  }
}

//Main function
void loop() {

  while (1) {
    lakePID();
  }
}


void lakePID() {
  const char _MAX_SPEED = 30;
  const char _GOAL_SPEED = 20;
  const double _KP = 0.5;
  const double _KD = 1;
  const double _LEFT_COEFFICIENT = 1.5;      // Left motor is farther from the sensor, so it needs more oomph
  const char _GOAL = 100;
  float red, green, blue;
  float red1, green1, blue1;
  static double _lastError = 0;

  // Take a color sensor reading (lower value means less color)
  pondSensor.getRGB(&red, &green, &blue);
  fishSensor.getRGB(&red1, &green1, &blue1);

  // Compute error
  double _error = blue - _GOAL;

  // Compute adjustment
  int _adjustment = _KP * _error + _KD * (_error - _lastError);
  _lastError = _error;

  // Adjust motors (B should go higher when blue is lower)
  driver.setMotorBPower(constrain(_GOAL_SPEED + _adjustment, 0, _GOAL_SPEED));
  driver.setMotorAPower(constrain(_GOAL_SPEED - _adjustment * _LEFT_COEFFICIENT, 0, _MAX_SPEED));

  Serial.print(int (red1));
  Serial.print("-");
  Serial.print(int (green1));
  Serial.print("-");
  Serial.print(int (blue1));
  Serial.println();git a


//  if(int (blue1) <= 75 && int (green1) >= 73){
//    driver.brakeAll();
//    //grabFish();
//  }
}

void grabFish() {
  HandServo.write(0);
  delay(1000);
  ArmServo.write(180);
  delay(1000);
  HandServo.write(120);
  delay(1000);
  ArmServo.write(0);
}

void ServoHome() {
  DumpServo.write(180);
  delay(100);
  HandServo.write(0);
  delay(100);
  ArmServo.write(0);
  delay(100);
}
