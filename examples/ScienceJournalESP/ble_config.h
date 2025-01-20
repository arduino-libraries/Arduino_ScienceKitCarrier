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

const int VERSION = 0x00000002;

#define BLE_CH_SERVICE               "0000"
#define BLE_CH_VERSION               "0001"

#define BLE_CH_CURRENT               "1001"
#define BLE_CH_VOLTAGE               "1002"
#define BLE_CH_RESISTA               "1003"
#define BLE_CH___LIGHT               "1004"
#define BLE_CH_PROXIMI               "1005"
#define BLE_CH_ACCELER               "1006"
#define BLE_CH_GYROSCO               "1007"
#define BLE_CH_MAGNETO               "1008"
#define BLE_CH_TEMPERA               "1009"
#define BLE_CH_PRESSUR               "1010"
#define BLE_CH_HUMIDIT               "1011"

#ifdef ARDUINO_NANO_RP2040_CONNECT
#define BLE_CH_SOUNDIN               "1012"
#define BLE_CH_SOUNDPI               "1013"
#endif

#define BLE_CH_FUNGEN1               "1014"
#define BLE_CH_DISTANC               "1015"
#define BLE_CH_INPUT_A               "1016"
#define BLE_CH_INPUT_B               "1017"
#define BLE_CH_AIR_QUA               "1018"
#define BLE_CH_EXTTEMP               "1019"
#define BLE_CH____PING               "1020"
#define BLE_CH_FUNGEN2               "1021"


#define SCIENCE_KIT_UUID(val) ("555a0003-" val "-467a-9538-01f0652c74e8")

/* 
 * SERVICE, VERSION 
 */

/* __________________________________________________________________SERVICE  */
BLEService                     service                    (SCIENCE_KIT_UUID(BLE_CH_SERVICE));
/* __________________________________________________________________VERSION  */
BLEUnsignedIntCharacteristic   versionCharacteristic      (SCIENCE_KIT_UUID(BLE_CH_VERSION), BLERead);

/* 
 * CURRENT, VOLTAGE, RESISTANCE 
 */

/* __________________________________________________________________CURRENT  */
BLEFloatCharacteristic         currentCharacteristic      (SCIENCE_KIT_UUID(BLE_CH_CURRENT), BLENotify);
/* __________________________________________________________________VOLTAGE  */
BLEFloatCharacteristic         voltageCharacteristic      (SCIENCE_KIT_UUID(BLE_CH_VOLTAGE), BLENotify);
/* ________________________________________________________________RESISTANCE */
BLEFloatCharacteristic         resistanceCharacteristic   (SCIENCE_KIT_UUID(BLE_CH_RESISTA), BLENotify);
/* ___________________________________________________________________LIGHT   */
BLECharacteristic              lightCharacteristic        (SCIENCE_KIT_UUID(BLE_CH___LIGHT), BLENotify, 4 * sizeof(long));
/* _________________________________________________________________PROXIMITY */
BLEUnsignedIntCharacteristic   proximityCharacteristic    (SCIENCE_KIT_UUID(BLE_CH_PROXIMI), BLENotify);

/* 
 * BMI270 & BMM150, 9dof imu, acceleration, gyroscope and magnetometer 
 */

/* ______________________________________________________________ACCELERATION */
BLECharacteristic              accelerationCharacteristic (SCIENCE_KIT_UUID(BLE_CH_ACCELER), BLENotify, 3 * sizeof(float));
/* _________________________________________________________________GYROSCOPE */
BLECharacteristic              gyroscopeCharacteristic    (SCIENCE_KIT_UUID(BLE_CH_GYROSCO), BLENotify, 3 * sizeof(float));
/* ______________________________________________________________MAGNETOMETER */
BLECharacteristic              magnetometerCharacteristic (SCIENCE_KIT_UUID(BLE_CH_MAGNETO), BLENotify, 3 * sizeof(float));

/* 
 * BME688 
 */

/* _______________________________________________________________TEMPERATURE */
BLEFloatCharacteristic         temperatureCharacteristic  (SCIENCE_KIT_UUID(BLE_CH_TEMPERA), BLENotify);
/* __________________________________________________________________PRESSURE */
BLEFloatCharacteristic         pressureCharacteristic     (SCIENCE_KIT_UUID(BLE_CH_PRESSUR), BLENotify);
/* __________________________________________________________________HUMIDITY */
BLEFloatCharacteristic         humidityCharacteristic     (SCIENCE_KIT_UUID(BLE_CH_HUMIDIT), BLENotify);
/* _______________________________________________________________AIR_QUALITY */
BLEFloatCharacteristic         airQualityCharacteristic   (SCIENCE_KIT_UUID(BLE_CH_AIR_QUA), BLENotify); // ***

/*
 * MICROPHONE
 */
#ifdef ARDUINO_NANO_RP2040_CONNECT
/* ___________________________________________________________SOUND_INTENSITY */
BLEUnsignedIntCharacteristic   sndIntensityCharacteristic (SCIENCE_KIT_UUID(BLE_CH_SOUNDIN), BLENotify); // *** !
/* ______!!! NOT AVAILABLE (should be delete?) !!!________________SOUND_PITCH */
BLEUnsignedIntCharacteristic   sndPitchCharacteristic     (SCIENCE_KIT_UUID(BLE_CH_SOUNDPI), BLENotify);
#endif

/*
 * INPUT A,B
 */

/* ___________________________________________________________________INPUT_A */
BLEUnsignedIntCharacteristic   inputACharacteristic       (SCIENCE_KIT_UUID(BLE_CH_INPUT_A), BLENotify);
/* ___________________________________________________________________INPUT_B */
BLEUnsignedIntCharacteristic   inputBCharacteristic       (SCIENCE_KIT_UUID(BLE_CH_INPUT_B), BLENotify);

/*
 * EXTERNAL TEMPERATURE
 */

/* ______________________________________________________EXTERNAL_TEMPERATURE */
BLEFloatCharacteristic         extTempCharacteristic      (SCIENCE_KIT_UUID(BLE_CH_EXTTEMP), BLENotify); // ***

/*
 * FUNCTION GENERATOR 1,2
 */

/* ______________________________________________________FUNCTION_GENERATOR_1 */
BLECharacteristic              funcGenOneCharacteristic   (SCIENCE_KIT_UUID(BLE_CH_FUNGEN1), BLENotify, 2 * sizeof(long));
/* ______________________________________________________FUNCTION_GENERATOR_2 */
BLECharacteristic              funcGenTwoCharacteristic   (SCIENCE_KIT_UUID(BLE_CH_FUNGEN2), BLENotify, 2 * sizeof(long));

/*
 * DISTANCE, PING
 */

/* __________________________________________________________________DISTANCE */
BLEFloatCharacteristic         distanceCharacteristic     (SCIENCE_KIT_UUID(BLE_CH_DISTANC), BLENotify); // ***
/* ______________________________________________________________________PING */
BLEFloatCharacteristic         pingCharacteristic         (SCIENCE_KIT_UUID(BLE_CH____PING), BLENotify); // ***


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