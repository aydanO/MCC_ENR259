/*
ENG 259 Robot

Authors:
Hanning, Eligh
Hoodak, Sam  
Johnson, Andrew  
Nguyen, Duc  
O' Brien, Aydan  
Serbetci, Suleyman 
Tretyak, Bogdan

Professor: Berl, Geoffrey 
*/

//Include libraries for motor driver and sensor array
#include <QTRSensors.h>
#include "TB67H420FTG.h"

// Line Sensor Properties
#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2

QTRSensorsAnalog qtra((unsigned char[]) {
  A9, A8, A7, A6, A5, A4, A3, A2        //Pin names on Teensy
}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// Motor Driver Properties
TB67H420FTG driver(4, 5, 8, 6, 7, 9);   //Pin numbers on Teensy

// PID Properties
const double KP = 0.02;
const double KD = 0.0;
double lastError = 0;
const int GOAL = 3500;
const unsigned char MAX_SPEED = 50;

//Global Variable for counting lines
int lineCount;

//Setup function to calibrate line sensor array
void setup() {
  driver.init();
  Serial.begin(9600);

  // Initialize line sensor array
  calibrateLineSensor();
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
    
    //Commented out for troubleshooting
    //    Serial.println(lineCount);
    //    Serial.print("->");
    //    Serial.println(count);
  }

  //If statement to turn robot right after passing total of 5 lines
  if (lineCount == 5) {
    RightTurn();
  }

  //Repeat right turn once we hit 8 lines total
  if (lineCount == 8) {
    RightTurn();
  }

  //Bring robot to stop after tracking 14 total grid lines
  if(lineCount == 14){
    driver.brakeAll();
  }

}


//Function to calibrate sensor, called in setup
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

//Function to turn right using timing of motor
void RightTurn() {
  driver.setMotorAPower(75);
  driver.setMotorBPower(-25);
  delay(500);
  driver.brakeAll();
  delay(500);
  lineCount++;
}
