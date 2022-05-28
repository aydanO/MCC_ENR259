#include <QTRSensors.h>
#include <Wire.h>
#include <WireIMXRT.h>
#include <WireKinetis.h>
#include "TB67H420FTG.h"
#include "Adafruit_TCS34725.h"

TB67H420FTG driver(5, 6, 9, 7, 8, 10);   //Pin numbers on Teensy
TB67H420FTG driver2(33, 34, 29, 35, 36, 30);   //Pin numbers on Teensy

int MAX_SPEED = 20;

void setup() {
  driver.init();
  driver2.init();
  Serial.begin(9600);

}

void loop() {
  driver2.setMotorBPower(-70);
  delay(275);
  driver2.brakeAll();
  //driver.setAllCoastPower(MAX_SPEED);
  driver2.setMotorAPower(70);
  delay(200);
  driver.brakeAll();
  delay(100);
  driver2.setMotorAPower(-70);
  delay(200);
  driver2.brakeAll();
  delay(100);
  driver2.setMotorBPower(70);
  delay(300);
  driver2.brakeAll();
  //driver.setAllCoastPower(MAX_SPEED);
  //delay(200);
  //driver.brakeAll();
  

}
