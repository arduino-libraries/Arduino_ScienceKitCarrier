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

#ifndef __ARDUINO_SCIENCEKITCARRIER_DEFINITIONS_H__
#define __ARDUINO_SCIENCEKITCARRIER_DEFINITIONS_H__

// Grove pads
#define INPUTA_PIN A0
#define INPUTB_PIN A1
#define ANALOGIN_DISABLED 0
#define UPDATE_ALL 0
#define UPDATE_INPUT_A 1
#define UPDATE_INPUT_B 2

// APDS9960
#define INT_APDS9960 9

// INA
const uint32_t SHUNT_MICRO_OHM{100000};    // check schematic R20
const uint16_t MAXIMUM_AMPS{1};            // 1A

// Resistence
#define RESISTANCE_PIN A2
#define RESISTOR_AUX  1000.0
#define REF_VOLTAGE 3.3
#define RESISTANCE_CALIBRATION_HIGH 200000
#define RESISTANCE_CALIBRATION_LOW 10

// Imu
#define G_EARTH 9.807

// Bme688
#define BME688_CS 10

// External temperature connected on input A
#define OW_PIN digitalPinToPinName(INPUTA_PIN)
#define EXTERNAL_TEMPERATURE_DISABLED -273.0; // absolute zero xD


// Errors
#define ERR_BEGIN_APDS -3
#define ERR_BEGIN_INA -4
#define ERR_BEGIN_IMU -5
#define ERR_BEGIN_BME -6
#define ERR_BEGIN_RESISTANCE -7
#define ERR_BEGIN_FUNCTION_GENERATOR_CONTROLLER -8
#define ERR_BEGIN_ULTRASONIC -9
#define ERR_BEGIN_EXTERNAL_TEMPERATURE -10




// Led
#define ACTIVITY_LED_OFF 0
#define ACTIVITY_LED_BLE 1
#define ACTIVITY_LED_PAIRING 2

// Update
#define ROUND_ROBIN_ENABLED 1
#define ROUND_ROBIN_DISABLED 0

#define NO_AUXILIARY_THREADS 0
#define START_AUXILIARY_THREADS 1
#define START_INTERNAL_AMBIENT_SENSOR 2     // bme688
#define START_EXTERNAL_AMBIENT_SENSOR 3     // ds18b20





// Servos
#define SERVO_A_PIN 3
#define SERVO_B_PIN 2



#endif