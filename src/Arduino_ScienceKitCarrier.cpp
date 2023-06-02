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

#include "Arduino_ScienceKitCarrier.h"


ScienceKitCarrier::ScienceKitCarrier(){
  inputA_pin = INPUTA_PIN;
  inputB_pin = INPUTB_PIN;
  inputA=0;
  inputB=0;

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

  
  thread_activity_led = new rtos::Thread();
  thread_update_bme = new rtos::Thread();

  activity_led_state = ACTIVITY_LED_OFF;
}





int ScienceKitCarrier::begin(){
  pinMode(LEDR,OUTPUT);
  digitalWrite(LEDR,LOW);
  pinMode(LEDG,OUTPUT);
  digitalWrite(LEDG,LOW);
  pinMode(LEDB,OUTPUT);
  digitalWrite(LEDB,LOW);


  Wire.begin();

  // let's start apds89960
  if (beginAPDS()!=0){
    return ERR_BEGIN_APDS;
  }

  // let's start ina
  if (beginINA()!=0){
    return ERR_BEGIN_INA;
  }

  // resistance pin
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

  // let's start activity led and bme688
  //startAuxiliaryThreads();
}





/********************************************************************/
/*                               Update                             */
/********************************************************************/

void ScienceKitCarrier::update(){
  updateAnalogInput();
  updateAPDS();
  updateINA();
  updateResistance();
  updateIMU();
}



/********************************************************************/
/*                          Analog Inputs                           */
/********************************************************************/

int ScienceKitCarrier::beginAnalogInput(){
  pinMode(inputA_pin, INPUT);
  pinMode(inputB_pin, INPUT);
  return 0;
}

void ScienceKitCarrier::updateAnalogInput(){
  inputA=analogRead(inputA_pin);
  inputB=analogRead(inputB_pin);
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
  for (int i=0; i<10; i++){
    resistance=REF_VOLTAGE*analogRead(resistance_pin)/1024.0;
    resistance=(RESISTOR_AUX*REF_VOLTAGE/resistance)-RESISTOR_AUX;
    if (opencircuit_resistance>resistance){
      opencircuit_resistance=resistance;
    }
  }
  updateResistance();
  return 0;
}

void ScienceKitCarrier::updateResistance(){
  resistance=REF_VOLTAGE*analogRead(resistance_pin)/1024.0;
  if (resistance<=0){
    resistance=-2.0;
  }
  else{
    resistance=(RESISTOR_AUX*REF_VOLTAGE/resistance)-RESISTOR_AUX;
    if (resistance<=RESISTANCE_CALIBRATION_LOW){
      resistance=0.0;
    }
    else{
      if (resistance>=opencircuit_resistance){
        resistance=-1.0;
      }
    }
  }
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

void ScienceKitCarrier::threadBME688(){
  beginBME688();
  while(true){
    updateBME688();
    rtos::ThisThread::sleep_for(1000);
  }
}





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

void ScienceKitCarrier::delay(unsigned long t){
  rtos::ThisThread::sleep_for(t);
}



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
/*                              Threads                             */
/********************************************************************/

void ScienceKitCarrier::startAuxiliaryThreads(){
  //thread_activity_led->start(mbed::callback(this, &ScienceKitCarrier::threadActivityLed));
  thread_update_bme->start(mbed::callback(this, &ScienceKitCarrier::threadBME688)); 
}