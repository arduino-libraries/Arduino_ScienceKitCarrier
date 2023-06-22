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

#ifndef __LED_RANGE_H__
#define __LED_RANGE_H__

#include "Arduino.h"
#include "led_matrix.h"

class LedRange:LedMatrix{
  private:
    uint8_t button;
    int value;
    uint8_t state;
    uint8_t previous;
    uint8_t range_1, range_2;
    bool status_refresh;
  public:
    LedRange(const uint8_t _button, const uint8_t _r1, const uint8_t _r2, const uint8_t _r3, const uint8_t _c1, const uint8_t _c2):LedMatrix(_r1, _r2, _r3, _c1, _c2){
      button = _button;
      pinMode(button,INPUT);
      value = 0;
      state = 0;
      previous = 0;
      range_1 = 0;
      range_2 = 0;
      status_refresh=true;
    }

    uint8_t checkPush(){
      value = analogRead(button);
      if ((value<100) || (value>1000)){
        return 2; //range 2
      }
      else{
        if (value>400){
          return 0; //nothing pressed
        }
        else{
          return 1; //range 1
        }
      }
      return 3;
    }

    void updateRange(uint8_t & r){
      if (r<2){
        r++;
      }
      else{
        r=0;
      }
    }
    
    void update(){
      state=checkPush();
      if (previous!=state){
        //nothing was pressed
        if (previous==0){
          //check range_1
          if (state==1){
            updateRange(range_1);
          }
          else{
            if (state==2){
              updateRange(range_2);
            }
          }
        }
      }
      alternate();
      previous=state;
    }

    uint8_t getRange1(){
      return pow(10,range_1);
    }

    uint8_t getRange2(){
      return pow(10,range_2);
    }

    void alternate(){
      if (status_refresh){
        set(range_1);
      }
      else{
        set(range_2+3);
      }
      status_refresh=!status_refresh;

    }
};



#endif