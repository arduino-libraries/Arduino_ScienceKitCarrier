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

#include "ble_config.h"
#include "Arduino_ScienceKitCarrier.h"
String name;
unsigned long lastNotify = 0;

ScienceKitCarrier science_kit;

rtos::Thread _t(osPriorityHigh);
rtos::Thread _tu;




void setup() {
  pinMode(8,OUTPUT);
  pinMode(7,OUTPUT);

  science_kit.begin(NO_AUXILIARY_THREADS); // Doesn't start the BME688 thread for the moment

  if (!BLE.begin()) {
    while(1);
  }

  String address = BLE.address();

  address.toUpperCase();

  name = "ScienceKit R3 - ";
  name += address[address.length() - 5];
  name += address[address.length() - 4];
  name += address[address.length() - 2];
  name += address[address.length() - 1];

  BLE.setLocalName(name.c_str());
  BLE.setDeviceName(name.c_str());
  BLE.setAdvertisedService(service);

  service.addCharacteristic(versionCharacteristic);
  service.addCharacteristic(currentCharacteristic);
  service.addCharacteristic(voltageCharacteristic);
  service.addCharacteristic(resistanceCharacteristic);
  service.addCharacteristic(lightCharacteristic);
  service.addCharacteristic(proximityCharacteristic);
  service.addCharacteristic(accelerationCharacteristic);
  service.addCharacteristic(gyroscopeCharacteristic);
  service.addCharacteristic(magnetometerCharacteristic);
  service.addCharacteristic(temperatureCharacteristic);
  service.addCharacteristic(pressureCharacteristic);
  service.addCharacteristic(humidityCharacteristic);
  service.addCharacteristic(sndIntensityCharacteristic);
  service.addCharacteristic(sndPitchCharacteristic);
  service.addCharacteristic(inputACharacteristic);
  service.addCharacteristic(inputBCharacteristic);

  versionCharacteristic.setValue(VERSION);

  BLE.addService(service);
  BLE.advertise();
  science_kit.startAuxiliaryThreads(); // start the BME688 and External Temperature Probe threads
  //_t.start(loop_data);
  _tu.start(update);
}

bool _is_connected = false;

void loop_data() {
  bool last_connected_status = _is_connected;
  while (1) {
    delay(1000);
    if (last_connected_status != _is_connected) {
      digitalWrite(7, _is_connected ? HIGH : LOW);
    }
  }
}

void update(void){
  while(1){
    digitalWrite(8,HIGH);
    science_kit.update(ROUND_ROBIN_ENABLED);
    digitalWrite(8,LOW);
    rtos::ThisThread::sleep_for(20);
  }
}

void loop() {
  
  BLEDevice central = BLE.central();
  if (central) {
    _is_connected = true;
    lastNotify=millis();
    while (central.connected()) {
      if (millis()-lastNotify>10){
        updateSubscribedCharacteristics();
        lastNotify=millis();
      }
    }
  } else {
    delay(100);
    _is_connected = false;
  }
}

void updateSubscribedCharacteristics() {
  if(currentCharacteristic.subscribed()) {
    currentCharacteristic.writeValue(science_kit.getCurrent());
  }

  if(voltageCharacteristic.subscribed()) {
    voltageCharacteristic.writeValue(science_kit.getVoltage());
  }
  
  if(resistanceCharacteristic.subscribed()) {
    resistanceCharacteristic.writeValue(science_kit.getResistance());  
  }
  
  if (lightCharacteristic.subscribed()) {
    long light[4];
    light[0] = science_kit.getRed();
    light[1] = science_kit.getGreen();
    light[2] = science_kit.getBlue();
    light[3] = science_kit.getClear();
    lightCharacteristic.writeValue((byte*)light, sizeof(light));
  }

  if (proximityCharacteristic.subscribed()) {
    proximityCharacteristic.writeValue(science_kit.getProximity());
  }
  
  if (accelerationCharacteristic.subscribed()) {
    float acceleration[3];
    acceleration[0] = science_kit.getAccelerationX();
    acceleration[1] = science_kit.getAccelerationY();
    acceleration[2] = science_kit.getAccelerationZ();
    accelerationCharacteristic.writeValue((byte*)acceleration, sizeof(acceleration));
  }

  if (gyroscopeCharacteristic.subscribed()) {
    float gyroscope[3];
    gyroscope[0] = science_kit.getAngularVelocityX();
    gyroscope[1] = science_kit.getAngularVelocityY();
    gyroscope[2] = science_kit.getAngularVelocityZ();
    gyroscopeCharacteristic.writeValue((byte*)gyroscope, sizeof(gyroscope));
  }

  if (magnetometerCharacteristic.subscribed()) {
    float magnetometer[3];
    magnetometer[0] = science_kit.getMagneticFieldX();
    magnetometer[1] = science_kit.getMagneticFieldY();
    magnetometer[2] = science_kit.getMagneticFieldZ();
    magnetometerCharacteristic.writeValue((byte*)magnetometer, sizeof(magnetometer));
  }
  
  if(temperatureCharacteristic.subscribed()) {
    temperatureCharacteristic.writeValue(science_kit.getTemperature());
  }
  
  if(pressureCharacteristic.subscribed()) {
    pressureCharacteristic.writeValue(science_kit.getPressure());
  }
  
  if(humidityCharacteristic.subscribed()) {
    humidityCharacteristic.writeValue(science_kit.getHumidity());
  }
  
  if(sndIntensityCharacteristic.subscribed()) {
    sndIntensityCharacteristic.writeValue(science_kit.getFrequency1());
  }

  if(sndPitchCharacteristic.subscribed()) {
    sndPitchCharacteristic.writeValue(science_kit.getExternalTemperature());
  }

  if (inputACharacteristic.subscribed()){
    inputACharacteristic.writeValue(science_kit.getInputA());
  }

  if (inputBCharacteristic.subscribed()){
    inputBCharacteristic.writeValue(science_kit.getInputB());
  }
}