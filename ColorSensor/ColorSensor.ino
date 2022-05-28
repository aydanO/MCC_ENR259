#include <Wire.h>
#include <WireIMXRT.h>
#include <WireKinetis.h>
#include "Adafruit_TCS34725.h"
int led = 13;
Adafruit_TCS34725 pondSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 fishSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  
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

void loop() {
  float red, green, blue;
  float red1, green1, blue1;
  delay(60);  // takes 50ms to read

  fishSensor.getRGB(&red, &green, &blue);
  pondSensor.getRGB(&red1, &green1, &blue1);
  

//  if (int(blue) < 100) {
//    digitalWrite(led, HIGH);
//  }
//  else if (int(blue) >= 100) {
//    digitalWrite(led, LOW);
 // }
  if (int(blue1) < 100) {
    digitalWrite(led, HIGH);
  }
  else if (int(blue1) >= 100) {
    digitalWrite(led, LOW);
  }
}
