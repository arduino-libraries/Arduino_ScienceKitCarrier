/*
  This file is part of the Arduino_GroveI2C_Ultrasonic library.
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

#ifndef __ARDUINO_SCIENCEKITCARRIER_H__
#define __ARDUINO_SCIENCEKITCARRIER_H__

#include <Arduino.h>
#include "WiFiNINA.h"
#include "mbed.h"
#include "rtos.h"
#include <Wire.h>
#include "Arduino_APDS9960.h"
#include "INA.h"
#include "bsec.h"
#include "Arduino_BMI270_BMM150.h"
#include "Arduino_GroveI2C_Ultrasonic.h"
#include <PDM.h>

#include "../../OneWireNg/src/platform/OneWireNg_PicoRP2040.h"  // forces to use gpio insted PIO hw
#define OneWireNg_CurrentPlatform OneWireNg_PicoRP2040
#include "drivers/DSTherm.h"
#include "utils/Placeholder.h"


#include "./utils/function_generator_controller.h"
#include "./utils/Arduino_ScienceKitCarrier_definitions.h"

static  Placeholder<OneWireNg_CurrentPlatform> ow;



class ScienceKitCarrier{
  private:
    uint8_t round_robin_index;

    uint8_t inputA_pin, inputB_pin;
    int inputA, inputB;
    uint8_t timer_inputA;

    APDS9960 * apds9960;
    int r,g,b,c, proximity;

    INA_Class * ina;
    float voltage, current;

    uint8_t resistance_pin;
    float resistance, opencircuit_resistance;

    Bsec * bme688;
    float temperature, pressure, humidity, airquality;
    uint8_t bme688_cs;

    BoschSensorClass * imu;
    float acceleration[3];
    float angular_velocity[3];
    float magnetic_field[3];

    FunctionGeneratorController * function_generator_controller;
    uint8_t frequency1, frequency2, phase1, phase2, range1, range2;

    Arduino_GroveI2C_Ultrasonic * ultrasonic;
    float distance, travel_time;
    bool ultrasonic_is_connected;

    bool external_temperature_is_connected;
    float external_temperature;



    uint microphone_rms, rms;
    static const char channels = MICROPHONE_CHANNELS;
    static const int frequency = MICROPHONE_FREQUENCY;


    rtos::Thread * thread_activity_led;
    rtos::Thread * thread_update_bme;
    rtos::Thread * thread_external_temperature;

    bool thread_bme_is_running;
    bool thread_ext_temperature_is_running;

    uint8_t activity_led_state;

  public:
    ScienceKitCarrier();

    int begin(const uint8_t auxiliary_threads=START_AUXILIARY_THREADS);
    void update(const bool roundrobin=false);  // this makes update on: analog in, imu, apds, ina, resistance, round robin enables one sensor update
    void startAuxiliaryThreads(const uint8_t auxiliary_threads=START_AUXILIARY_THREADS);



    void delay(unsigned long t); // you must use this instead delay, due threads usage



    /* Blink red alert */
    void errorTrap(const int error_code=0);

    /* Activity led */
    void threadActivityLed();
    void setActivityLed(const int led_state=ACTIVITY_LED_OFF);



    /* Analog input connected to Grove connectors A and B */
    int beginAnalogInput();
    void updateAnalogInput(const uint8_t input_to_update=UPDATE_ALL);
    int getInputA();              // 0-1024
    int getInputB();              // 0-1024



    /* APDS9960, used for light temperature/ambient light and proximity */
    int beginAPDS();
    void updateAPDS();
    int getProximity();           // 0-255
    int getRed();                 // 0-255
    int getGreen();               // 0-255
    int getBlue();                // 0-255
    int getClear();               // 0-255



    /* INA, current and voltage measurements */
    int beginINA();
    void updateINA();
    float getVoltage();           // Volt
    float getCurrent();           // Ampere



    /* Resistance */
    int beginResistance();
    void updateResistance();
    float getResistance();        // Ohm



    /* BME688, temperature, pressure and humidity */
    int beginBME688();
    void updateBME688();
    float getTemperature();       // Celsius
    float getPressure();          // hPascal
    float getHumidity();          // Percentage
    float getAirQuality();        // index, if good it is 25.0
    void threadBME688();          // thread used to update BME688 automatically in multithread mode



    /* BMI270 & BMM150, 9dof imu, acceleration, gyroscope and magnetometer */
    int beginIMU();
    void updateIMU();

    void getAcceleration(float & x, float & y, float & z);
    float getAccelerationX();
    float getAccelerationY();
    float getAccelerationZ();

    void getAngularVelocity(float & x, float & y, float & z);
    float getAngularVelocityX();
    float getAngularVelocityY();
    float getAngularVelocityZ();

    void getMagneticField(float & x, float & y, float & z);
    float getMagneticFieldX();
    float getMagneticFieldY();
    float getMagneticFieldZ();



    /* Functions generator data */
    int beginFrequencyGeneratorData();
    void updateFrequencyGeneratorData();
    uint8_t getFrequency1();
    uint8_t getFrequency2();
    uint8_t getPhase1();
    uint8_t getPhase2();
    uint8_t getRange1();
    uint8_t getRange2();



    /* Ultrasonic sensor */
    int beginUltrasonic();
    void updateUltrasonic();
    float getDistance();        // meters
    float getTravelTime();      // microseconds
    bool getUltrasonicIsConnected();



    /* External temperature probe - Dallas DS18B20 */
    int beginExternalTemperature();
    void updateExternalTemperature();
    float getExternalTemperature();     // celsius
    bool getExternalTemperatureIsConnected();
    void threadExternalTemperature();



    /* Microphone - onboard PDM */
    int beginMicrophone();
    void updateMicrophone();  
    static void updateMicrophoneDataBuffer();  // interrupt function
    uint getMicrophoneRMS();

    static short sampleBuffer[MICROPHONE_BUFFER_SIZE]; //must be public
    static volatile int samplesRead;


};




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