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

#ifdef ARDUINO_NANO_RP2040_CONNECT
short ScienceKitCarrier::sampleBuffer[MICROPHONE_BUFFER_SIZE];
volatile int ScienceKitCarrier::samplesRead;
#endif


ScienceKitCarrier::ScienceKitCarrier(){
  round_robin_index=0;

  inputA_pin = INPUTA_PIN;
  inputB_pin = INPUTB_PIN;
  inputA=0;
  inputB=0;
  timer_inputA = 0;

  apds9960 = new APDS9960(Wire,INT_APDS9960);
  proximity=0;
  r=0;
  g=0;
  b=0;
  c=0;

  ina = new INA_Class();
  voltage=0.0;
  current=0.0;

  resistance_pin = RESISTANCE_PIN;
  opencircuit_resistance = RESISTANCE_CALIBRATION_HIGH;

  bme688 = new Bsec();
  temperature=0.0;
  pressure=0.0;
  humidity=0.0;
  airquality=0.0;
  bme688_cs = BME688_CS;

  imu = new BoschSensorClass();
  acceleration[0]=0.0;
  acceleration[1]=0.0;
  acceleration[2]=0.0;
  angular_velocity[0]=0.0;
  angular_velocity[1]=0.0;
  angular_velocity[2]=0.0;
  magnetic_field[0]=0.0;
  magnetic_field[1]=0.0;
  magnetic_field[2]=0.0;

  function_generator_controller = new FunctionGeneratorController();
  frequency1=0;
  frequency2=0;
  phase1=0;
  phase2=0;
  range1=0;
  range2=0;

  ultrasonic = new Arduino_GroveI2C_Ultrasonic();
  distance=0.0;
  travel_time=0.0;
  ultrasonic_is_connected=false;

  external_temperature=EXTERNAL_TEMPERATURE_DISABLED;
  external_temperature_is_connected=false;

  #ifdef ARDUINO_NANO_RP2040_CONNECT
  microphone_rms=0;
  rms=0;
  #endif

  round_robin_index=0;
  
  #ifdef ARDUINO_NANO_RP2040_CONNECT
  thread_activity_led = new rtos::Thread();
  thread_update_bme = new rtos::Thread();
  thread_external_temperature = new rtos::Thread();
  #endif

  thread_bme_is_running = false;
  thread_ext_temperature_is_running = false;

  activity_led_state = ACTIVITY_LED_OFF;
}





/********************************************************************/
/*                              Begin                               */
/********************************************************************/

int ScienceKitCarrier::begin(const uint8_t auxiliary_threads){
  pinMode(LEDR,OUTPUT);
  pinMode(LEDG,OUTPUT);
  pinMode(LEDB,OUTPUT);

  #ifdef ARDUINO_NANO_RP2040_CONNECT
  digitalWrite(LEDR,LOW);
  digitalWrite(LEDG,LOW);
  digitalWrite(LEDB,LOW);
  #endif

  #ifdef ESP32
  digitalWrite(LEDR,HIGH);
  digitalWrite(LEDG,HIGH);
  digitalWrite(LEDB,HIGH);
  #endif


  Wire.begin();

  // most of begin functions return always 0, it is a code-style or future implementation

  // let's start apds9960
  if (beginAPDS()!=0){
    return ERR_BEGIN_APDS;
  }

  // let's start ina
  if (beginINA()!=0){
    return ERR_BEGIN_INA;
  }
  

  // let's start resistance measurement
  if (beginResistance()!=0){
    return ERR_BEGIN_RESISTANCE;
  }
  
  // let's start imu
  if (beginIMU()!=0){
    return ERR_BEGIN_IMU;
  }

  // let's start function generator controller
  if (beginFrequencyGeneratorData()!=0){
    return ERR_BEGIN_FUNCTION_GENERATOR_CONTROLLER;
  }

  // let's start microphone (PDM on Arduino Nano RP2040 Connect)
  #ifdef ARDUINO_NANO_RP2040_CONNECT
  if (beginMicrophone()!=0){
    return ERR_BEGIN_MICROPHONE;
  }
  #endif

  // let's start ultrasonic and check if it is connected
  /* WIP
  if (beginUltrasonic()!=0){
    return ERR_BEGIN_ULTRASONIC;
  }
  */

  // let's start bme688 and external ds18b20 probe
  //WIP startAuxiliaryThreads(auxiliary_threads);
}





/********************************************************************/
/*                               Update                             */
/********************************************************************/

void ScienceKitCarrier::update(const bool roundrobin){
  #ifdef ARDUINO_NANO_RP2040_CONNECT
  updateMicrophone();                        // about 1ms when MICROPHONE_BUFFER_SIZE is 512
  #endif
  if (!roundrobin){
    updateAnalogInput();
    updateAPDS();
    updateINA();
    updateResistance();
    updateIMU();
    updateFrequencyGeneratorData();

    // update external
    //WIP updateUltrasonic();
  }
  else{
    //WIP
    switch (round_robin_index){
      case 0: 
        if (thread_ext_temperature_is_running){
          updateAnalogInput(UPDATE_INPUT_B); // it is very fast (about 1ms)
        }
        else{
          updateAnalogInput();               // updated both A and B channel
        }
        updateFrequencyGeneratorData();      // less than 1ms
        break;
      case 1:
        updateAPDS();                        // about 5ms
        break;
      case 2:
        updateINA();                         // about 3ms
        break;
      case 3:
        updateResistance();                  // about 1ms
        // WIP updateUltrasonic();                  // requires about 5ms when not connected
        break;
      case 4:
        updateIMU();                         // heavy task, 13ms
        break;
      default:
        break;
    }
    round_robin_index++;
    if (round_robin_index>4){
      round_robin_index=0;
    }
  }

}



/********************************************************************/
/*                          Analog Inputs                           */
/********************************************************************/

int ScienceKitCarrier::beginAnalogInput(){
  pinMode(inputA_pin, INPUT);
  pinMode(inputB_pin, INPUT);
  return 0;
}

void ScienceKitCarrier::updateAnalogInput(const uint8_t input_to_update){
  if ((input_to_update==UPDATE_INPUT_A)||(input_to_update==UPDATE_ALL)){
    /*   WIP
    if (!getExternalTemperatureIsConnected()){
      inputA=analogRead(inputA_pin);
    }
    else{
      inputA=ANALOGIN_DISABLED;
    } 
    */
  }
  if ((input_to_update==UPDATE_INPUT_B)||(input_to_update==UPDATE_ALL)){
    inputB=analogRead(inputB_pin);
  }
}

int ScienceKitCarrier::getInputA(){
  return inputA;
}

int ScienceKitCarrier::getInputB(){
  return inputB;
}





/********************************************************************/
/*                             APDS9960                             */
/********************************************************************/
int ScienceKitCarrier::beginAPDS(){
  if (!apds9960->begin()) {
    return ERR_BEGIN_APDS;
  }
  return 0;
}
void ScienceKitCarrier::updateAPDS(){
  if (apds9960->proximityAvailable()){
    proximity=apds9960->readProximity();
  }
  if (apds9960->colorAvailable()){
    apds9960->readColor(r,g,b,c);
  }
}

int ScienceKitCarrier::getProximity(){
  return proximity;
}

int ScienceKitCarrier::getRed(){
  return r;
}

int ScienceKitCarrier::getGreen(){
  return g;
}

int ScienceKitCarrier::getBlue(){
  return b;
}

int ScienceKitCarrier::getClear(){
  return c;
}





/********************************************************************/
/*                               INA                                */
/********************************************************************/

int ScienceKitCarrier::beginINA(){
  if (!ina->begin(MAXIMUM_AMPS, SHUNT_MICRO_OHM)){
    return ERR_BEGIN_INA;
  }

  ina->setBusConversion(8500);             // Maximum conversion time 8.244ms
  ina->setShuntConversion(8500);           // Maximum conversion time 8.244ms
  ina->setAveraging(128);                  // Average each reading n-times
  ina->setMode(INA_MODE_CONTINUOUS_BOTH);  // Bus/shunt measured continuously
  ina->alertOnBusOverVoltage(true, 5000);  // Trigger alert if over 5V on bus
  return 0;
}

void ScienceKitCarrier::updateINA(){
  voltage = ina->getBusMilliVolts(0);
  voltage = voltage/1000.0;
  current = ina->getBusMicroAmps(0);
  current = current/1000000.0;
}

float ScienceKitCarrier::getVoltage(){
  return voltage;
}

float ScienceKitCarrier::getCurrent(){
  return current;
}





/********************************************************************/
/*                            Resistance                            */
/********************************************************************/

int ScienceKitCarrier::beginResistance(){
  pinMode(resistance_pin,INPUT);
  // search for minimum open circuit resistance
  for (int i=0; i<20; i++){
    resistance = getResistanceMeasureVolts();
    resistance = (RESISTOR_AUX*REF_VOLTAGE/resistance)-RESISTOR_AUX;
    if (opencircuit_resistance>resistance){
      opencircuit_resistance = resistance;
    }
    delay(5);
  }
  opencircuit_resistance=opencircuit_resistance-1000;
  updateResistance();
  return 0;
}

void ScienceKitCarrier::updateResistance(){
  resistance = getResistanceMeasureVolts();
  if (resistance <= 0){
    resistance = -2.0;
  }
  else{
    resistance = (RESISTOR_AUX*REF_VOLTAGE/resistance)-RESISTOR_AUX;
    if (resistance <= RESISTANCE_CALIBRATION_LOW){
      resistance = 0.0;
    }
    else{
      if (resistance>=opencircuit_resistance){
        resistance = -1.0;
      }
    }
  }
}

float ScienceKitCarrier::getResistanceMeasureVolts(){
  float value = 0.0;
  #ifdef ARDUINO_NANO_RP2040_CONNECT
    value = REF_VOLTAGE*analogRead(resistance_pin)/ADC_RESOLUTION;
  #endif
  #ifdef ESP32
    value = analogReadMilliVolts(resistance_pin)/1000.0;
  #endif
  return value;
}



float ScienceKitCarrier::getResistance(){
  return resistance;
}





/********************************************************************/
/*                              BME688                              */
/********************************************************************/

int ScienceKitCarrier::beginBME688(){
  SPI.begin();
  bme688->begin(bme688_cs,SPI);

  if (bme688->bsecStatus != 0){
    return ERR_BEGIN_BME;
  }
  if (bme688->bme68xStatus != 0){
    return ERR_BEGIN_BME;
  }

  bsec_virtual_sensor_t sensorList[13] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_STABILIZATION_STATUS,
    BSEC_OUTPUT_RUN_IN_STATUS,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_GAS_PERCENTAGE
  };

  bme688->updateSubscription(sensorList, 13, BSEC_SAMPLE_RATE_CONT);
  return 0;
}

void ScienceKitCarrier::updateBME688(){
  if (bme688->run()){
    temperature=bme688->temperature;
    pressure=(bme688->pressure)/100.0;
    humidity=bme688->humidity;
    airquality=bme688->iaq;
  }
}

float ScienceKitCarrier::getTemperature(){
  return temperature;
}

float ScienceKitCarrier::getPressure(){
  return pressure;
}

float ScienceKitCarrier::getHumidity(){
  return humidity;
}

float ScienceKitCarrier::getAirQuality(){
  return airquality;
}

#ifdef ARDUINO_NANO_RP2040_CONNECT
void ScienceKitCarrier::threadBME688(){
  beginBME688();
  while(1){
    updateBME688();
    rtos::ThisThread::sleep_for(1000);
  }
}
#endif





/********************************************************************/
/*                               IMU                                */
/********************************************************************/

int ScienceKitCarrier::beginIMU(){
  if (!imu->begin()){
    return ERR_BEGIN_IMU;
  }
  return 0;
}
void ScienceKitCarrier::updateIMU(){
  if (imu->accelerationAvailable()){
    imu->readAcceleration(acceleration[0], acceleration[1], acceleration[2]);
    // change scale g -> m/s^2 and rotate the TF according the board symbol
    acceleration[0] = -acceleration[0]*G_EARTH;
    acceleration[1] = -acceleration[1]*G_EARTH;
    acceleration[2] = -acceleration[2]*G_EARTH;
  }
  if (imu->gyroscopeAvailable()){
    imu->readGyroscope(angular_velocity[0], angular_velocity[1], angular_velocity[2]);
    // fix orientation
    angular_velocity[0] = -angular_velocity[0];
    angular_velocity[1] = -angular_velocity[1];
    angular_velocity[2] = -angular_velocity[2];
  }
  if (imu->magneticFieldAvailable()){
    imu->readMagneticField(magnetic_field[0], magnetic_field[1], magnetic_field[2]);
    // change TF orientation
    magnetic_field[0] = -magnetic_field[0];
    magnetic_field[1] = magnetic_field[1];
    magnetic_field[2] = magnetic_field[2];
  }
}

void ScienceKitCarrier::getAcceleration(float & x, float & y , float & z){
  x = acceleration[0];
  y = acceleration[1];
  z = acceleration[2];
}

float ScienceKitCarrier::getAccelerationX(){
  return acceleration[0];
}

float ScienceKitCarrier::getAccelerationY(){
  return acceleration[1];
}

float ScienceKitCarrier::getAccelerationZ(){
  return acceleration[2];
}

void ScienceKitCarrier::getAngularVelocity(float & x, float & y, float & z){
  x = angular_velocity[0];
  y = angular_velocity[1];
  z = angular_velocity[2];
}

float ScienceKitCarrier::getAngularVelocityX(){
  return angular_velocity[0];
}

float ScienceKitCarrier::getAngularVelocityY(){
  return angular_velocity[1];
}

float ScienceKitCarrier::getAngularVelocityZ(){
  return angular_velocity[2];
}

void ScienceKitCarrier::getMagneticField(float & x, float & y, float & z){
  x = magnetic_field[0];
  y = magnetic_field[1];
  z = magnetic_field[2];
}

float ScienceKitCarrier::getMagneticFieldX(){
  return magnetic_field[0];
}

float ScienceKitCarrier::getMagneticFieldY(){
  return magnetic_field[1];
}

float ScienceKitCarrier::getMagneticFieldZ(){
  return magnetic_field[2];
}

/********************************************************************/
/*                             delay                                */
/********************************************************************/

#ifdef ARDUINO_NANO_RP2040_CONNECT
void ScienceKitCarrier::delay(unsigned long t){
  rtos::ThisThread::sleep_for(t);
}
#endif



/********************************************************************/
/*                   LEDs: errors and activity                      */
/********************************************************************/

void ScienceKitCarrier::errorTrap(const int error_code){
  if (error_code!=0){
    while(1){
      for (int i=0; i<-error_code; i++){
        digitalWrite(LEDR,HIGH);
        delay(100);
        digitalWrite(LEDR,LOW);
        delay(500);
      }
      delay(3000);
    }
  }
  else{
    while(1){
      digitalWrite(LEDR,HIGH);
      delay(100);
      digitalWrite(LEDR,LOW);
      delay(500);
    }
  }
}

#ifdef ARDUINO_NANO_RP2040_CONNECT
void ScienceKitCarrier::threadActivityLed(){
  while(1){
    switch (activity_led_state){
      case ACTIVITY_LED_OFF:
        digitalWrite(LEDB,LOW);
        digitalWrite(LEDG,LOW);
        break;
      case ACTIVITY_LED_BLE:               // blue breathing effect
        digitalWrite(LEDG, LOW);
        for(int i=255; i>0; i--){
          analogWrite(LEDB, i);
          rtos::ThisThread::sleep_for(10);
        }
        for(int i=0; i<255; i++){
          analogWrite(LEDB, i);
          rtos::ThisThread::sleep_for(10);
        }
        rtos::ThisThread::sleep_for(100);
        break;
      case ACTIVITY_LED_PAIRING:            // blue-green flashing
        for(int i = 255; i>0; i=i-2){
          analogWrite(LEDG,i);
          rtos::ThisThread::sleep_for(1);
        }
        for(int i = 0; i<255; i=i+2){
          analogWrite(LEDG,i);
          rtos::ThisThread::sleep_for(1);
        }
        for(int i = 255; i>0; i=i-2){
          analogWrite(LEDB,i);
          rtos::ThisThread::sleep_for(1);
        }
        for(int i = 0; i<255; i=i+2){
          analogWrite(LEDB,i);
          rtos::ThisThread::sleep_for(1);
        }
        digitalWrite(LEDG, LOW);
        digitalWrite(LEDB, LOW);
        break;
      default:                              // any other value turns off leds
        digitalWrite(LEDB,LOW);
        digitalWrite(LEDG,LOW);
    }  
  }
}
#endif

void ScienceKitCarrier::setActivityLed(const int led_state){
  activity_led_state=led_state;
}



/********************************************************************/
/*                  Function Generator Controller                   */
/********************************************************************/

int ScienceKitCarrier::beginFrequencyGeneratorData(){
  function_generator_controller->begin();
  return 0;
}

void ScienceKitCarrier::updateFrequencyGeneratorData(){
  function_generator_controller->updateData();
  function_generator_controller->getData(frequency1, range1, phase1, frequency2, range2, phase2);
}

uint8_t ScienceKitCarrier::getFrequency1(){
  return frequency1;
}

uint8_t ScienceKitCarrier::getFrequency2(){
  return frequency2;
}

uint8_t ScienceKitCarrier::getPhase1(){
  return phase1;
}

uint8_t ScienceKitCarrier::getPhase2(){
  return phase2;
}

uint8_t ScienceKitCarrier::getRange1(){
  return range1;
}

uint8_t ScienceKitCarrier::getRange2(){
  return range2;
}



/********************************************************************/
/*                        Ultrasonic Sensor                         */
/********************************************************************/

int ScienceKitCarrier::beginUltrasonic(){
  ultrasonic->begin();
  updateUltrasonic();
}

void ScienceKitCarrier::updateUltrasonic(){
  if (ultrasonic->checkConnection()){
    ultrasonic_is_connected=true;
    distance=ultrasonic->getMeters();
    travel_time=ultrasonic->getTravelTime();
  }
  else{
    ultrasonic_is_connected=false;
  }
}

float ScienceKitCarrier::getDistance(){
  return distance;
}

float ScienceKitCarrier::getTravelTime(){
  return travel_time;
}

bool ScienceKitCarrier::getUltrasonicIsConnected(){
  return ultrasonic_is_connected;
}



/********************************************************************/
/*                    External Temperature Probe                    */
/********************************************************************/
//WIP

#ifdef ARDUINO_NANO_RP2040_CONNECT
int ScienceKitCarrier::beginExternalTemperature(){
  new (&ow) OneWireNg_CurrentPlatform(OW_PIN, false);
  DSTherm drv(ow);
  return 0;
}

void ScienceKitCarrier::updateExternalTemperature(){
  float temperature;
  pinMode(OW_PIN,INPUT);
  
  DSTherm drv(ow);
  drv.convertTempAll(DSTherm::MAX_CONV_TIME, false);  

  static Placeholder<DSTherm::Scratchpad> scrpd;
  OneWireNg::ErrorCode ec = drv.readScratchpadSingle(scrpd);
  
  if (ec == OneWireNg::EC_SUCCESS) {
    if (scrpd->getAddr()!=15){
      external_temperature_is_connected=false;
      external_temperature = EXTERNAL_TEMPERATURE_DISABLED;
    }
    else{
      external_temperature_is_connected=true;
      long temp = scrpd->getTemp();
      int sign=1;
      if (temp < 0) {
        temp = -temp;
        sign=-1;
      }
      temperature = (temp/1000)+(temp%1000)*0.001;
      external_temperature = sign*temperature;
    }   
  }
}


float ScienceKitCarrier::getExternalTemperature(){
  return external_temperature;
}

bool ScienceKitCarrier::getExternalTemperatureIsConnected(){
  return external_temperature_is_connected;
}

void ScienceKitCarrier::threadExternalTemperature(){
  beginExternalTemperature();
  while(1){
    updateExternalTemperature();
    updateAnalogInput(UPDATE_INPUT_A);
    rtos::ThisThread::sleep_for(1000);
  }
}
#endif



/********************************************************************/
/*                             Microphone                           */
/********************************************************************/
#ifdef ARDUINO_NANO_RP2040_CONNECT
int ScienceKitCarrier::beginMicrophone(){
  PDM.setGain(50);
  PDM.onReceive(updateMicrophoneDataBuffer);
  if(!PDM.begin(channels, frequency)){
    return ERR_BEGIN_MICROPHONE;
  }
  return 0;
}

void ScienceKitCarrier::updateMicrophone(){
  if (samplesRead) {
    // Calculate the RMS of data buffer
    rms=0;
    for (int i=0; i<samplesRead; i++){
      rms=rms+(sampleBuffer[i]*sampleBuffer[i]);
    }
    rms=rms/samplesRead;
    microphone_rms=sqrt(rms);
    samplesRead = 0;
  }
}

void ScienceKitCarrier::updateMicrophoneDataBuffer(){
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}

uint ScienceKitCarrier::getMicrophoneRMS(){
  return microphone_rms;
}
#endif




/********************************************************************/
/*                              Threads                             */
/********************************************************************/
#ifdef ARDUINO_NANO_RP2040_CONNECT

void ScienceKitCarrier::startAuxiliaryThreads(const uint8_t auxiliary_threads){
  //thread_activity_led->start(mbed::callback(this, &ScienceKitCarrier::threadActivityLed));    //left for legacy on prototypes and maybe future implementations

  // start bme688 thread
  if ((auxiliary_threads==START_AUXILIARY_THREADS)||(auxiliary_threads==START_INTERNAL_AMBIENT_SENSOR)){
    if (!thread_bme_is_running){
      thread_update_bme->start(mbed::callback(this, &ScienceKitCarrier::threadBME688)); 
    }
    thread_bme_is_running=true;
  }

  // start ds18b20 thread
  if ((auxiliary_threads==START_AUXILIARY_THREADS)||(auxiliary_threads==START_EXTERNAL_AMBIENT_SENSOR)){
    if (!thread_ext_temperature_is_running){
      thread_external_temperature->start(mbed::callback(this, &ScienceKitCarrier::threadExternalTemperature));
    }
    thread_ext_temperature_is_running=true;
  }
}

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