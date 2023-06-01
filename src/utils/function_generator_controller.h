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

#ifndef __FUNCTION_GENERATOR_CONTROLLER_H__
#define __FUNCTION_GENERATOR_CONTROLLER_H__

#include "Arduino.h"
#include "Wire.h"


#define FG_VERSION 'V'
#define FG_DATA 'D'

#define DEFAULT_I2C_ADDRESS 0x2F

class FunctionGeneratorController{
  private:
    uint8_t address;
    TwoWire& wire;
    uint8_t version[2];
    uint8_t data[6]; // frequency1, scale1, phase1, frequency2, scale2, phase2

  public:
    FunctionGeneratorController(uint8_t _address=DEFAULT_I2C_ADDRESS, TwoWire & _wire=Wire):wire(_wire){
      address=_address;
    }

    void begin(){
      wire.begin();
    }

    String getVersion(){
      wire.beginTransmission(address);
      wire.write(FG_VERSION);
      wire.endTransmission();
      wire.requestFrom(address,2);
      wire.readBytes(version,2);
      return String(version[0])+"."+String(version[1]);
    }

    void updateData(){
      wire.beginTransmission(address);
      wire.write(FG_DATA);
      wire.endTransmission();
      wire.requestFrom(address,6);
      wire.readBytes(data,6);
    }

    void getData(uint8_t & f1, uint8_t & s1, uint8_t & p1, uint8_t & f2, uint8_t & s2, uint8_t & p2){
      updateData();
      f1=data[0];
      s1=data[1];
      p1=data[2];
      f2=data[3];
      s2=data[4];
      p2=data[5];
    }
};

#endif