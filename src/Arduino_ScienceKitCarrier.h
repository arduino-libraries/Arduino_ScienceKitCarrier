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

#ifdef ARDUINO_NANO_RP2040_CONNECT
#include "WiFiNINA.h"
#include "mbed.h"
#include "rtos.h"
#include <PDM.h>
#endif

#include <Wire.h>
#include "Arduino_APDS9960.h"
#include "INA.h"

#include "bsec2.h"

#include "Arduino_BMI270_BMM150.h"

#ifdef ARDUINO_NANO_RP2040_CONNECT
#include "../../OneWireNg/src/platform/OneWireNg_PicoRP2040.h"  // forces to use gpio instead PIO hw
#define OneWireNg_CurrentPlatform OneWireNg_PicoRP2040
#endif
#ifdef ESP32
#include "OneWireNg_CurrentPlatform.h"
#endif
#include "drivers/DSTherm.h"
#include "utils/Placeholder.h"


#include "./utils/function_generator_controller.h"
#include "./utils/Arduino_ScienceKitCarrier_definitions.h"

static  Placeholder<OneWireNg_CurrentPlatform> ow;


#ifdef ARDUINO_NANO_RP2040_CONNECT
#define wire_lock wire_mutex.lock()
#define wire_unlock wire_mutex.unlock()
#endif

#ifdef ESP32
#define wire_lock while(!xSemaphoreTake(wire_semaphore, 5)){}
#define wire_unlock xSemaphoreGive(wire_semaphore)
#endif


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

    Bsec2 * bme688;
    float temperature, pressure, humidity, airquality;
    uint8_t bme688_cs;

    BoschSensorClass * imu;
    float acceleration[3];
    float angular_velocity[3];
    float magnetic_field[3];

    FunctionGeneratorController * function_generator_controller;
    uint8_t frequency1, frequency2, phase1, phase2, range1, range2;

    float ultrasonic_measure,distance, travel_time;
    uint32_t ultrasonic_data;
    bool ultrasonic_is_connected;

    bool external_temperature_is_connected;
    float external_temperature;


    #ifdef ARDUINO_NANO_RP2040_CONNECT
    uint microphone_rms, rms;
    static const char channels = MICROPHONE_CHANNELS;
    static const int frequency = MICROPHONE_FREQUENCY;

    rtos::Thread * thread_status_led;
    rtos::Thread * thread_update_bme;
    rtos::Thread * thread_external_temperature;
    rtos::Thread * thread_ultrasonic;
    rtos::Mutex wire_mutex;
    #endif

    #ifdef ESP32
    TaskHandle_t thread_internal_temperature;
    TaskHandle_t thread_external_temperature;
    TaskHandle_t thread_ultrasonic;
    TaskHandle_t thread_led;
    SemaphoreHandle_t wire_semaphore;
    #endif

    bool thread_bme_is_running;
    bool thread_ext_temperature_is_running;
    bool thread_ultrasonic_is_running;
    bool thread_led_is_running;

    uint8_t status_led_state;
    bool enable_led_red, enable_led_green, enable_led_blue;
    unsigned long led_time_base;

    void requestUltrasonicUpdate();
    void retriveUltrasonicUpdate();

  public:
    ScienceKitCarrier();

    int begin(const uint8_t auxiliary_threads=START_AUXILIARY_THREADS);
    void update(const bool roundrobin=false);  // this makes update on: analog in, imu, apds, ina, resistance, round robin enables one sensor update

    void startAuxiliaryThreads(const uint8_t auxiliary_threads=START_AUXILIARY_THREADS);

    /* Blink red alert */
    void errorTrap(const int error_code=0);

    /* Status led */
    #ifdef ESP32
    void setStatusLed(const int led_state=STATUS_LED_OFF);
    void threadStatusLed();
    static void freeRTOSStatusLed(void * pvParameters);
    #endif



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
    float getResistanceMeasureVolts();  // Volt



    /* BME688, temperature, pressure and humidity */
    int beginBME688();
    void updateBME688();
    float getTemperature();       // Celsius
    float getPressure();          // hPascal
    float getHumidity();          // Percentage
    float getAirQuality();        // index, if good it is 25.0
    void threadBME688();          // thread used to update BME688 automatically in multithread mode
    #ifdef ESP32
    static void freeRTOSInternalTemperature(void * pvParameters);
    #endif


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
    void updateUltrasonic();
    float getDistance();        // meters
    float getTravelTime();      // microseconds
    bool getUltrasonicIsConnected();
    void threadUltrasonic();
    #ifdef ESP32
    static void freeRTOSUltrasonic(void * pvParameters);
    #endif



    /* External temperature probe - Dallas DS18B20 */
    int beginExternalTemperature();
    void updateExternalTemperature();
    float getExternalTemperature();     // celsius
    bool getExternalTemperatureIsConnected();
    void threadExternalTemperature();
    #ifdef ESP32
    static void freeRTOSExternalTemperature(void * pvParameters);
    #endif

    #ifdef ARDUINO_NANO_RP2040_CONNECT
    /* Microphone - onboard PDM */
    int beginMicrophone();
    void updateMicrophone();  
    static void updateMicrophoneDataBuffer();  // interrupt function
    uint getMicrophoneRMS();

    static short sampleBuffer[MICROPHONE_BUFFER_SIZE]; //must be public
    static volatile int samplesRead;
    #endif

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