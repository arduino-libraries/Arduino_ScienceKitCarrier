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

#ifndef __BLE_CONFIG_H__
#define __BLE_CONFIG_H__

#include <ArduinoBLE.h>

const int VERSION = 0x00000001;

#define SCIENCE_KIT_UUID(val) ("555a0003-" val "-467a-9538-01f0652c74e8")
BLEService                     service                    (SCIENCE_KIT_UUID("0000"));
BLEUnsignedIntCharacteristic   versionCharacteristic      (SCIENCE_KIT_UUID("0001"), BLERead);
BLEFloatCharacteristic         currentCharacteristic      (SCIENCE_KIT_UUID("1001"), BLENotify);
BLEFloatCharacteristic         voltageCharacteristic      (SCIENCE_KIT_UUID("1002"), BLENotify);
BLEFloatCharacteristic         resistanceCharacteristic   (SCIENCE_KIT_UUID("1003"), BLENotify);
BLECharacteristic              lightCharacteristic        (SCIENCE_KIT_UUID("1004"), BLENotify, 4 * sizeof(long));
BLEUnsignedIntCharacteristic   proximityCharacteristic    (SCIENCE_KIT_UUID("1005"), BLENotify);
BLECharacteristic              accelerationCharacteristic (SCIENCE_KIT_UUID("1006"), BLENotify, 3 * sizeof(float));
BLECharacteristic              gyroscopeCharacteristic    (SCIENCE_KIT_UUID("1007"), BLENotify, 3 * sizeof(float));
BLECharacteristic              magnetometerCharacteristic (SCIENCE_KIT_UUID("1008"), BLENotify, 3 * sizeof(float));
BLEFloatCharacteristic         temperatureCharacteristic  (SCIENCE_KIT_UUID("1009"), BLENotify);
BLEFloatCharacteristic         pressureCharacteristic     (SCIENCE_KIT_UUID("1010"), BLENotify);
BLEFloatCharacteristic         humidityCharacteristic     (SCIENCE_KIT_UUID("1011"), BLENotify);
BLEUnsignedIntCharacteristic   sndIntensityCharacteristic (SCIENCE_KIT_UUID("1012"), BLENotify);
BLEUnsignedIntCharacteristic   sndPitchCharacteristic     (SCIENCE_KIT_UUID("1013"), BLENotify);
BLEUnsignedIntCharacteristic   inputACharacteristic     (SCIENCE_KIT_UUID("1016"), BLENotify);
BLEUnsignedIntCharacteristic   inputBCharacteristic     (SCIENCE_KIT_UUID("1017"), BLENotify);

#endif



/***
 *                       _       _                                    
 *         /\           | |     (_)                                   
 *        /  \   _ __ __| |_   _ _ _ __   ___                         
 *       / /\ \ | '__/ _` | | | | | '_ \ / _ \                        
 *      / ____ \| | | (_| | |_| | | | | | (_) |                       
 *     /_/____\_\_| _\__,_|\__,_|_|_| |_|\___/ ___ _     _____  ____  
 *      / ____|    (_)                     | |/ (_) |   |  __ \|___ \ 
 *     | (___   ___ _  ___ _ __   ___ ___  | ' / _| |_  | |__) | __) |
 *      \___ \ / __| |/ _ \ '_ \ / __/ _ \ |  < | | __| |  _  / |__ < 
 *      ____) | (__| |  __/ | | | (_|  __/ | . \| | |_  | | \ \ ___) |
 *     |_____/ \___|_|\___|_| |_|\___\___| |_|\_\_|\__| |_|  \_\____/ 
 *                                                                    
 *                                                                    
 */