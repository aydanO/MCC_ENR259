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
#include <ezButton.h>

// Line Sensor Properties
#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2

ezButton button(32);  // create ezButton object that attach to pin 32;

Adafruit_TCS34725 pondSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 fishSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

const unsigned char MAX_SPEED = 50;

QTRSensorsAnalog qtra((unsigned char[]) {
  A9, A8, A7, A6, A3, A2, A1, A0        //Pin names on Teensy
}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// Motor Driver Properties
TB67H420FTG driver(5, 6, 9, 7, 8, 10);
TB67H420FTG driver2(33, 34, 29, 35, 36, 30);

// PID Properties
const double KP = 0.02;
const double KD = 0.0;
double lastError = 0;
const int GOAL = 500;

//Global Variable for counting lines
int lineCount = 0;

//Setup function to calibrate line sensor array
void setup() {
  driver.init();
  driver2.init();
  Serial.begin(9600);

  button.setDebounceTime(50); // set debounce time to 50 milliseconds

  calibrateLineSensor();

  while (!button.isPressed())
    button.loop(); // MUST call the loop() function first

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

  if (lineCount == 0) {
    driver.setMotorAPower(50);
    driver.setMotorBPower(50);
    delay(250);
    lineCount ++;
  }

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

  if (lineCount == 2) {
    CircleFollow(8, 7);
    lineCount ++;
  }

  if (lineCount == 9) {
    CircleFollow(18, 7);
    lineCount++;

  }

  else if (lineCount == 12) {
    CircleFollow(10, 7);
    lineCount++;

  }

}


void CircleFollow(int z, int i) {
  int _lineCount = 0;
  int _sensorVal;
  bool Tripped = 0;

  while (_lineCount <= z) {
    qtra.readLine(sensorValues);
    _sensorVal = sensorValues[i];


    if (_sensorVal > 250) {
      if (Tripped == 0) {
        _lineCount ++;
        Tripped = 1;
        digitalWrite(LED_BUILTIN, HIGH);
      }
    }

    else {
      digitalWrite(LED_BUILTIN, LOW);
      Tripped = 0;
    }


    float red, green, blue;
    float red1, green1, blue1;

    pondSensor.getRGB(&red, &green, &blue);
    fishSensor.getRGB(&red1, &green1, &blue);
    Serial.print(int(blue));

    if (int(blue) <= 100) {
      CircleTurn();
    }
    else if (int(blue) > 100) {
      driver.setMotorAPower(25);
      driver.setMotorBPower(25);
    }
    else if (int(blue1) > 100) {
      grabFish();
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


  void CircleTurn() {
    driver.setMotorAPower(-6);
    driver.setMotorBPower(25);
  }

  void grabFish() {
    driver2.setMotorBPower(-70);
    delay(275);
    driver2.brakeAll();
    driver2.setMotorAPower(70);
    delay(200);
    driver2.brakeAll();
    driver.setAllCoastPower(MAX_SPEED);
    delay(200);
    driver.brakeAll();
  }
