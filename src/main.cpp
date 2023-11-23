#include <Arduino.h>
#include <Kalman.h> // Kalman filter
#include <LSM6DS3.h> // IMU driver
#include <Control.h> // laser control algorithm
#include <DataWrite.h> // for writing measurement data

void setup() {
  Serial.begin(9600);
  while(!Serial);
}

void loop() {
  Serial.println("Hello World!");
  delay(1000);
}