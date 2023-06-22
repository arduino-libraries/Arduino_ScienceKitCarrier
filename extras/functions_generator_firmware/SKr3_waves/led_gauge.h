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


#ifndef __LED_GAUGE_H__
#define __LED_GAUGE_H__

#include "Arduino.h"
#include "led_array.h"

#define ADC_RES 511
#define NLED 8


class LedGauge: public LedArray{
  private:
    uint8_t pot;
    int value;
    int adc_step;
    int led;
    int gauge_value;
    int min, step;
    bool two;
  public:
    LedGauge(const uint8_t _pot, const int _min, const int _step, const uint8_t _r1, const uint8_t _r2, const uint8_t _r3, const uint8_t _c1, const uint8_t _c2, const uint8_t _c3, const bool _two=false):LedArray(_r1,_r2,_r3,_c1,_c2,_c3){
      pot=_pot;
      min=_min;
      step=_step;
      pinMode(pot, INPUT);
      value=0;
      adc_step = ADC_RES/NLED;
      two=_two;
    }

    //Call it everytime you want to read the pot and update leds
    void refresh(){
      value=analogRead(pot);
      if (value<=ADC_RES+100){  // pots create strange effects near 0
        led = (value+10)/adc_step; //+10 is for fine tuning
        if (two==true){
          led+=20;
        }
      }
      else{
        led = 0;
        if (two==true){
          led+=20;
        }
      }
      set(led);
      if (two==true){
        gauge_value=step*(led-20)+min;
      }
      else{
        gauge_value=step*led+min;
      }
    }

    // Value of the pot
    int getRaw(){
      return value;
    }

    // Value scaled
    int getGaugeValue(){
      return gauge_value;
    }

    // Leds indicator
    int getLed(){
      return led;
    }
};





#endif