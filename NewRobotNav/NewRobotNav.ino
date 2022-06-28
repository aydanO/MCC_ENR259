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

#define DEBUG_LAKE_PID 0

//          R   G  B
// Y Fish   110 79 57
// R Fish   119 67 63
// No Fish  108 73 66
#define IS_YELLOW_FISH int(blue) <= 60 && int(green) >= 75
#define IS_RED_FISH int(red) >= 115

#define DUMP_DOWN 177
#define HAND_OPEN 0
#define ARM_MIDWAY 90
#define HAND_CLOSED 120
#define ARM_UP 0
#define ARM_DOWN 145

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
#include <WireKinetis.h>
#include <Servo.h>

Adafruit_TCS34725 fishSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 pondSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  // emitter is controlled by digital pin 2

ezButton button(32);

// line sensor
QTRSensorsAnalog qtra((unsigned char[]) {
        A9, A8, A7, A6, A3, A2, A1, A0        //Pin names on Teensy
}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
int BlackThreshold = 250;

// Objects
TB67H420FTG driver(5, 6, 9, 7, 8, 10);   //Pin numbers on Teensy
Servo DumpServo;
Servo ArmServo;
Servo HandServo;

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

//    colorSensorTest();

    /* ###### Run through sequence of steps ######## */

    // Find first line
    while (!isIntersection()) {
        PID(3000);
    }

    // Follow first lake
    char lineCount = 0;
    while (lineCount <= 10) {
        // Check for black
        if (isSensorSeeingBlack(7)) {
            lineCount++;
            waitUntilSensorNotSeeingBlack(7);
        }

        // Call PID for lake
        lakePID();

        // Check for fish
        checkForFish();
    }

    // (UNTESTED)
    // After first lake, travel to next lake (sample numbers, fill in as needed)
    pidFollowForNIntersections(1000, 4);

    // Follow second lake (UNTESTED)
    lineCount = 0;
    while (lineCount <= 21) {
        // Check for black
        if (isSensorSeeingBlack(7)) {
            lineCount++;
            waitUntilSensorNotSeeingBlack(7);
        }

        // Call PID for lake
        lakePID();

        // Check for fish
        checkForFish();
    }


    // Stop doin' stuff
    driver.brakeAll();
}

//Main function
void loop() {

//    //medium pond
//    driver.setMotorAPower(50);
//    driver.setMotorBPower(50);
//    delay(1000);
//    lakePID(10, 7);
//
//    //big pond
//    PID(1000, 4);
//    lakePID(21, 7);
//
//    //Small Pond
//    PID(6500, 2);
//    lakePID(10, 7);

}

void checkForFish() {
    float red, green, blue;
    fishSensor.getRGB(&red, &green, &blue);

    if (IS_YELLOW_FISH) {
        driver.brakeAll();
        grabFish();
    } else if (IS_RED_FISH) {
        skipFish();
    }
}


void PID(int Goal) {
    const double KP = 0.02;
    const double KD = 0.0;
    const int MAX_SPEED = 50;
    double lastError = 0;

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
}


void lakePID() {
    const char MAX_SPEED = 30;
    const char GOAL_SPEED = 20;
    const double KP = 0.3;
    const double KD = 1;
    const double RIGHT_COEFFICIENT = 1.5;      // Left motor is farther from the sensor, so it needs more oomph
    const char GOAL = 96;
    static float red, green, blue;
    static double lastError = 0;
    static double error = 0;

//    while (x <= pondLines) {
//        qtra.readLine(sensorValues);
//        _sensorVal = sensorValues[i];
//
//            Serial.println(_sensorVal);
//            delay(100);

//        if (_sensorVal > 250) {
//            if (Tripped == 0) {
//                x++;
//                Tripped = true;
//                digitalWrite(LED_BUILTIN, HIGH);
//            }
//        } else {
//            digitalWrite(LED_BUILTIN, LOW);
//            Tripped = false;
//        }
//
//         Take a color sensor reading (lower value means less color)
    static unsigned long lastRead = millis() - 50;
    if (millis() - lastRead > 50) {
        pondSensor.getRGB(&red, &green, &blue);
        lastRead = millis();
    }
//        fishSensor.getRGB(&red1, &green1, &blue1);

    // Compute error
    error = blue - GOAL;

    // Compute adjustment
    int adjustment = KP * error + KD * (error - lastError);
    lastError = error;

    // Adjust motors (B is the Right motor)
    driver.setMotorBPower(constrain(GOAL_SPEED + adjustment, 0, MAX_SPEED));
    driver.setMotorAPower(constrain(GOAL_SPEED - adjustment * RIGHT_COEFFICIENT, 0, MAX_SPEED));

#if DEBUG_LAKE_PID
    Serial.print("sen:");
    Serial.print(blue);
    Serial.print(" err:");
    Serial.print(error);
    Serial.print(" adj:");
    Serial.println(adjustment);
#endif
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
    
    driver.setMotorBPower(15);
    driver.setMotorAPower(30);
    delay(1000);
    driver.brakeAll();
    HandServo.write(HAND_CLOSED);
    delay(500);
    ArmServo.write(ARM_UP);
    delay(2000);
    HandServo.write(HAND_OPEN);
    delay(500);
    ArmServo.write(ARM_DOWN);
    delay(1000);
}

void ServoHome() {
    
    DumpServo.write(DUMP_DOWN);
    delay(100);
    HandServo.write(HAND_OPEN);
    delay(100);
    ArmServo.write(ARM_DOWN);
//    ArmServo.write(ARM_MIDWAY);
    delay(1000);
}

void skipFish() {
    unsigned int startTime = millis();

    ArmServo.write(ARM_MIDWAY);
    while (millis() - startTime < 3000) {
        lakePID();
    }
    ArmServo.write(ARM_DOWN);
    delay(500);
}

bool isIntersection() {
    qtra.readLine(sensorValues);
    if (sensorValues[0] >= BlackThreshold && sensorValues[7] >= BlackThreshold) {
        return true;
    }
    return false;
}

bool isSensorSeeingBlack(int sensor) {
    const int BLACK_THRESHOLD = 250;

    qtra.readLine(sensorValues);
    if (sensorValues[sensor] > BLACK_THRESHOLD) {
        return true;
    }
    return false;
}

void waitUntilSensorNotSeeingBlack(int sensor) {
    const int BLACK_THRESHOLD = 250;

    do {
        qtra.readLine(sensorValues);
        // Do nothing (Motors should be moving still)
    } while (sensorValues[sensor] > BLACK_THRESHOLD);

}

void pidFollowForNIntersections(int Goal, char numIntersections) {
    char lineCount = 0;
    while (lineCount <= numIntersections) {
        while (!isIntersection()) {
            PID(Goal);
        }
        lineCount++;
    }
}

void colorSensorTest() {

    while(true) {
        float red, green, blue;
        fishSensor.getRGB(&red, &green, &blue);

        Serial.print("R: ");
        Serial.print(red, DEC);
        Serial.print(" ");
        Serial.print("G: ");
        Serial.print(green, DEC);
        Serial.print(" ");
        Serial.print("B: ");
        Serial.println(blue, DEC);
        delay(100);
    }
}

//bool intersection(int Goal, int numInt) {
//    if (sensorValues[0] >= BlackThreshold && sensorValues[7] >= BlackThreshold) {
//        while (sensorValues[0] >= BlackThreshold && sensorValues[7] >= BlackThreshold) {
//            PID(Goal, numInt);
//        }
//        return 1;
//    }
//    return 0;
//}
