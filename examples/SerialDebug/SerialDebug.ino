/*
    This file is part of the Arduino_ScienceKitCarrier library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/

// This examples print all data from Arduino Science Kit R3

#include "Arduino_ScienceKitCarrier.h"

ScienceKitCarrier science_kit;


void setup() {
  Serial.begin(115200);
  while(!Serial);
  science_kit.begin(START_AUXILIARY_THREADS);
}


void loop() {
  science_kit.update(ROUND_ROBIN_ENABLED);
  Serial.print(science_kit.getAccelerationX());
  Serial.print("\t");
  Serial.print(science_kit.getAccelerationY());
  Serial.print("\t ");
  Serial.print(science_kit.getAccelerationZ());
  Serial.print("\t| ");
  Serial.print(science_kit.getAngularVelocityX());
  Serial.print("\t");
  Serial.print(science_kit.getAngularVelocityY());
  Serial.print("\t");
  Serial.print(science_kit.getAngularVelocityZ());
  Serial.print("\t| ");
  Serial.print(science_kit.getMagneticFieldX());
  Serial.print("\t");
  Serial.print(science_kit.getMagneticFieldY());
  Serial.print("\t");
  Serial.print(science_kit.getMagneticFieldZ());
  Serial.print("\t|| ");
  Serial.print(science_kit.getProximity());
  Serial.print("\t");
  Serial.print(science_kit.getRed());
  Serial.print("\t");
  Serial.print(science_kit.getGreen());
  Serial.print("\t");
  Serial.print(science_kit.getBlue());
  Serial.print("\t|| ");
  Serial.print(science_kit.getPhase1());
  Serial.print("\t");
  Serial.print(science_kit.getFrequency1());
  Serial.print("\t");
  Serial.print(science_kit.getRange1());
  Serial.print("\t| ");
  Serial.print(science_kit.getPhase2());
  Serial.print("\t");
  Serial.print(science_kit.getFrequency2());
  Serial.print("\t");
  Serial.print(science_kit.getRange2());
  Serial.print("\t|| ");
  Serial.print(science_kit.getInputA());
  Serial.print("\t");
  Serial.print(science_kit.getInputB());
  Serial.print("\t|| ");
  Serial.print(science_kit.getVoltage());
  Serial.print("\t");
  Serial.print(science_kit.getCurrent());
  Serial.print("\t| ");
  Serial.print(science_kit.getResistance());
  Serial.print("\t|| ");

  Serial.print(science_kit.getTemperature());
  Serial.print("\t");
  Serial.print(science_kit.getPressure());
  Serial.print("\t");
  Serial.print(science_kit.getHumidity());
  Serial.print("\t");
  Serial.print(science_kit.getAirQuality());
  Serial.print("\t|| ");

  Serial.print(science_kit.getExternalTemperature());
  Serial.print("\t");
  Serial.print(science_kit.getExternalTemperatureIsConnected());
  Serial.print("\t|| ");

  Serial.print(science_kit.getUltrasonicIsConnected());
  Serial.print("\t");
  Serial.print(science_kit.getDistance());
  Serial.print("\t");
  Serial.print(science_kit.getTravelTime());
  Serial.print("\t|| ");

  // Microphone is only availble on Arduino Nano RP2040 Connect edition
  #ifdef ARDUINO_NANO_RP2040_CONNECT
  Serial.print(science_kit.getMicrophoneRMS());
  Serial.print("\t|| ");
  #endif
  
  Serial.print("\n");
  
  delay(10);
}
