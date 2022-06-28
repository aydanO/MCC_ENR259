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
int lineCount = 0;
int BlackThreshold = 250;


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

  ServoHome();
  calibrateLineSensor();


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
  driver.setMotorAPower(50);
  driver.setMotorBPower(50);
  delay(1000);
  lakePID(10, 7);

  //big pond
  PID(1000, 4);
  lakePID(21, 7);

  //Small Pond
  PID(6500, 2);
  lakePID(10, 7);

}


void PID(int Goal, int LineCountGoal) {
  const double KP = 0.02;
  const double KD = 0.0;
  double lastError = 0;
  static bool Tripped = 0;
  int lineCount = 0;
  int count = 0;

  while (lineCount <= LineCountGoal) {
    // Get line position
    unsigned int position = qtra.readLine(sensorValues);

    // Compute error from line
    int error = Goal - position;

    // Compute motor adjustment
    int adjustment = KP * error + KD * (error - lastError);

    // Store error for next increment
    lastError = error;


    // Adjust motors
    driver.setMotorBPower(constrain(MAX_SPEED - adjustment, 0, MAX_SPEED));
    driver.setMotorAPower(constrain(MAX_SPEED + adjustment, 0, MAX_SPEED));

    //Call to intersection function
    //    if (intersection(Goal, LineCountGoal, &lineCount)) {
    //      lineCount++;
    //    }
    //    if (sensorValues[0] >= BlackThreshold && sensorValues[7] >= BlackThreshold) {
    //      while (sensorValues[0] >= BlackThreshold && sensorValues[7] >= BlackThreshold) {
    //        PID(Goal, LineCountGoal);
    //      }
    //      lineCount ++;
    //    }
    for (unsigned char i = 0; i < 8; i ++) {
      qtra.readLine(sensorValues);
      int _sensorVal = sensorValues[i];
      if (_sensorVal > 250) {
        count ++;
      }
    }
    if (count >= 4) {
      if (Tripped == 0) {
        lineCount ++;
        Tripped = 1;
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println(lineCount);
      }
    }
    else {
      digitalWrite(LED_BUILTIN, LOW);
      Tripped = 0;
    }
  }
}


void lakePID(int pondLines, int i) {
  const char _MAX_SPEED = 30;
  const char _GOAL_SPEED = 20;
  const double _KP = 0.3;
  const double _KD = 1;
  const double _LEFT_COEFFICIENT = 1.5;      // Left motor is farther from the sensor, so it needs more oomph
  const char _GOAL = 96;
  float red, green, blue;
  float red1, green1, blue1;
  static double _lastError = 0;
  int _sensorVal;
  int x = 0;
  bool Tripped = 0;
  static double _error = 0;

  while (x <= pondLines) {
    qtra.readLine(sensorValues);
    _sensorVal = sensorValues[i];
    //
    //    Serial.println(_sensorVal);
    //    delay(100);

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
    //if(int (blue) >= 100){
    _error = blue - _GOAL;
    //    }

    // Compute adjustment
    int _adjustment = _KP * _error + _KD * (_error - _lastError);
    _lastError = _error;

    // Adjust motors (B should go higher when blue is lower)
    driver.setMotorBPower(constrain(_GOAL_SPEED + _adjustment, 0, _GOAL_SPEED));
    driver.setMotorAPower(constrain(_GOAL_SPEED - _adjustment * _LEFT_COEFFICIENT, 0, _MAX_SPEED));

    if (int (blue1) <= 63) {
      //driver.brakeAll();
      //grabFish();
    }
    else if (int (red1) >= 93) {
      //skipFish();
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
  DumpServo.write(177);
  delay(100);
  HandServo.write(107);
  delay(100);
  //ArmServo.write(145);
  ArmServo.write(50);
  delay(3000);
}

void skipFish() {
  ArmServo.write(90);
  delay(2000);
  ArmServo.write(150);
  delay(2000);
}

//bool intersection(int Goal, int numInt, int *lineCount) {
//  if (sensorValues[0] >= BlackThreshold && sensorValues[7] >= BlackThreshold) {
//    while (sensorValues[0] >= BlackThreshold && sensorValues[7] >= BlackThreshold) {
//      PID(Goal, numInt, lineCount);
//    }
//    return 1;
//  }
//  return 0;
//}
