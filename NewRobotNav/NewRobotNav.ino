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
#include <ezButton.h>

Adafruit_TCS34725 fishSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 pondSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2

const unsigned char MAX_SPEED = 50;

ezButton button (32);

// line sensor
QTRSensorsAnalog qtra((unsigned char[]) {
  A9, A8, A7, A6, A3, A2, A1, A0        //Pin names on Teensy
}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// Motor Driver Properties
TB67H420FTG driver(5, 6, 9, 7, 8, 10);   //Pin numbers on Teensy

// PID Properties
const double KP = 0.02;
const double KD = 0.0;
double lastError = 0;


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

  button.setDebounceTime(50); // set debounce time to 50 milliseconds

  calibrateLineSensor();
  ServoHome();

  while (!button.isPressed())
    button.loop();

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

  //medium pond
  PID(500, 1);
  lakePID(7, 7);

  //big pond
  PID(500, 5);
  lakePID(21, 7);

  //Small Pond
  PID(6500, 2);
  lakePID(10, 7);

}


void PID(int GOAL, int LineCountGoal) {
  const double KP = 0.02;
  const double KD = 0.0;
  double lastError = 0;
  int lineCount = 0;

  while (lineCount <= LineCountGoal) {
    // Get line position
    unsigned int position = qtra.readLine(sensorValues);

    // Compute error from line
    int error = GOAL - position;

    // Compute motor adjustment
    int adjustment = KP * error + KD * (error - lastError);

    // Store error for next increment
    lastError = error;


    // Adjust motors
    driver.setMotorAPower
    (constrain(MAX_SPEED - adjustment, 0, MAX_SPEED));
    driver.setMotorBPower(constrain(MAX_SPEED + adjustment, 0, MAX_SPEED));

    //Initialize varaible for counting of lines(each time, not total lines like lineCount)
    int count = 0;

    for (unsigned char i = 0; i < NUM_SENSORS; i++)
    {
      if (sensorValues[i] > 250) {
        count++;
      }
    }

    //If statement for robot to go straight until it passes 4 grid lines
    if (count >= 4) {
      driver.setAllCoastPower(MAX_SPEED);
      lineCount++;
      delay(100);
    }
  }
}


void lakePID(int pondLines, int i) {
  const char _MAX_SPEED = 30;
  const char _GOAL_SPEED = 20;
  const double _KP = 0.35;
  const double _KD = 1;
  const double _LEFT_COEFFICIENT = 1.5;      // Left motor is farther from the sensor, so it needs more oomph
  const char _GOAL = 100;
  float red, green, blue;
  float red1, green1, blue1;
  static double _lastError = 0;
  int _sensorVal;
  int x = 0;
  bool Tripped = 0;

  while (x <= pondLines) {
    qtra.readLine(sensorValues);
    _sensorVal = sensorValues[i];

    if (_sensorVal > 250) {
      if (Tripped == 0) {
        x ++;
        Tripped = 1;
        digitalWrite(LED_BUILTIN, HIGH);
      }
    }

    else {
      digitalWrite(LED_BUILTIN, LOW);
      Tripped = 0;
    }

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

    if (int (blue1) <= 63) {
      driver.brakeAll();
      grabFish();
    }
    else if (int (red1) >= 93) {
      skipFish();
    }
  }
}
void calibrateLineSensor() {
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 3000; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
}

void grabFish() {
  driver.setMotorBPower(10);
  driver.setMotorAPower(30);
  delay(800);
  driver.brakeAll();
  HandServo.write(120);
  delay(1000);
  ArmServo.write(0);
  delay(3000);
  HandServo.write(0);
  delay(1000);
  ArmServo.write(150);
  delay(3000);
}

void ServoHome() {
  DumpServo.write(180);
  delay(100);
  HandServo.write(0);
  delay(100);
  ArmServo.write(150);
  delay(3000);
}

void skipFish() {
  ArmServo.write(90);
  delay(2000);
  ArmServo.write(150);
  delay(2000);
}
