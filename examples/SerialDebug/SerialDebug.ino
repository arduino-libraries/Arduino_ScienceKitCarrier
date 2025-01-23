/*
  This file is part of the Arduino_ScienceKitCarrier library.
  Copyright (c) 2023 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Arduino_ScienceKitCarrier.h"

ScienceKitCarrier science_kit;


void setup() {
  Serial.begin(115200);
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

  Serial.print("\n");
  delay(10);
}