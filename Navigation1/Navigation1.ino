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

// Line Sensor Properties
#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2
#define redpin 3
#define greenpin 5
#define bluepin 6
#define commonAnode true
byte gammatable[256];

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

const unsigned char MAX_SPEED = 50;


QTRSensorsAnalog qtra((unsigned char[]) {
  A9, A8, A7, A6, A3, A2, A1, A0        //Pin names on Teensy
}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// Motor Driver Properties
TB67H420FTG driver(4, 5, 8, 6, 7, 9);   //Pin numbers on Teensy

// PID Properties
const double KP = 0.02;
const double KD = 0.0;
double lastError = 0;
const int GOAL = 500;

//Global Variable for counting lines
int lineCount;

//Setup function to calibrate line sensor array
void setup() {
  driver.init();
  Serial.begin(9600);
  //  delay(5000);
  //  Serial.println("Howdy");

  calibrateLineSensor();

  TwoWire *teensyWire = &Wire;
  teensyWire->setSDA(25);
  teensyWire->setSCL(24);
  teensyWire->setClock(400000);
  if (tcs.begin(TCS34725_ADDRESS, teensyWire)) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (true);
  }


  // use these three pins to drive an LED
#if defined(ARDUINO_ARCH_ESP32)
  ledcAttachPin(redpin, 1);
  ledcSetup(1, 12000, 8);
  ledcAttachPin(greenpin, 2);
  ledcSetup(2, 12000, 8);
  ledcAttachPin(bluepin, 3);
  ledcSetup(3, 12000, 8);
#else
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
#endif


  for (int i = 0; i < 256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }
    //Serial.println(gammatable[i]);
  }
}

//Main function
void loop() {

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


  if (lineCount == 5) {
    CircleFollow(70);
    lineCount++;

  }

  else if (lineCount == 9) {
    CircleFollow(110);
    lineCount++;
    
  }

  else if (lineCount == 12) {
    RightTurn();
    CircleFollow(150);
    lineCount++;
  }

}


void CircleFollow(int z) {

  for (int v = 0; v <= z; v++) {

    digitalWrite(LED_BUILTIN, HIGH);

    float red, green, blue;

    tcs.setInterrupt(false);  // turn on LED

    delay(60);  // takes 50ms to read

    tcs.getRGB(&red, &green, &blue);
    Serial.print(int(blue));


    if (int(blue) < 100) {
      CircleTurn();
    }
    else if (int(blue) > 100) {
      driver.setMotorAPower(25);
      driver.setMotorBPower(25);

    }
  }
  digitalWrite(LED_BUILTIN, LOW);
  
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

void RightTurn(){
  driver.setMotorAPower(30);
  driver.setMotorBPower(10);
}
