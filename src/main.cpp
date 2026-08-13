#include <Arduino.h>

#include "stabilizer.h"
#include "gps.h"

void setup() {
  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("SMART WALLET");

  stabilizerBegin();

  gpsBegin();

  Serial.println("SYSTEM READY");
}

void loop() {
  stabilizerUpdate();
  gpsUpdate();
  stabilizerPrint();
}