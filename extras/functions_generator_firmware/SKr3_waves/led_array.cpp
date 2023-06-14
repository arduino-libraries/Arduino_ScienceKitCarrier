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

#include "Arduino.h"
#include "led_array.h"

LedArray::LedArray(const uint8_t _r1, const uint8_t _r2, const uint8_t _r3, const uint8_t _c1, const uint8_t _c2, const uint8_t _c3){
  r1=_r1;
  r2=_r2;
  r3=_r3;
  c1=_c1;
  c2=_c2;
  c3=_c3;

  pinMode(r1,OUTPUT);
  pinMode(r2,OUTPUT);
  pinMode(r3,OUTPUT);
  pinMode(c1,OUTPUT);
  pinMode(c2,OUTPUT);
  pinMode(c3,OUTPUT);

  set(0,0,0,0,0,0);
}

/* frequency selector
   +-----+-----+------+
   | ROW | COL | LED  |
   +-----+-----+------+
   | A_1 | A_1 |  90  |
   | A_2 | A_1 |  70  |
   | A_3 | A_1 |  80  |
   | A_1 | A_2 |  60  |
   | A_2 | A_2 |  40  |
   | A_3 | A_2 |  50  | 
   | A_1 | A_3 |  30  |
   | A_2 | A_3 |  10  |
   | A_3 | A_3 |  20  |   
   +-----+-----+------+  
*/

/* frequency selector 2 mid-march
   +-----+-----+------+
   | ROW | COL | LED  |
   +-----+-----+------+
   | B_1 | B_1 |  90  |
   | B_2 | B_1 |  80  |
   | B_3 | B_1 |  70  |
   | B_1 | B_2 |  60  |
   | B_2 | B_2 |  50  |
   | B_3 | B_2 |  40  | 
   | B_1 | B_3 |  30  |
   | B_2 | B_3 |  10  |
   | B_3 | B_3 |  20  |   
   +-----+-----+------+  
*/

void LedArray::set(const int x){
  switch(x){
    case 0: 
      set(1,0,1,0,0,1);
      break;
    case 1: 
      set(1,1,0,0,0,1);
      break;
    case 2: 
      set(0,1,1,0,0,1);  
      break;
    case 3: 
      set(1,0,1,0,1,0);
      break;
    case 4: 
      set(1,1,0,0,1,0);
      break;
    case 5: 
      set(0,1,1,0,1,0);
      break;
    case 6: 
      set(1,0,1,1,0,0);
      break;
    case 7: 
      set(1,1,0,1,0,0);
      break;
    case 8: 
      set(0,1,1,1,0,0);
      break;

    case 20:  
      set(1,1,0,0,0,1);
      break;
    case 21: 
      set(1,0,1,0,0,1);
      break;
    case 22: 
      set(0,1,1,0,0,1);  
      break;
    case 23: 
      set(1,1,0,0,1,0);
      break;
    case 24: 
      set(1,0,1,0,1,0);
      break;
    case 25: 
      set(0,1,1,0,1,0);
      break;
    case 26: 
      set(1,1,0,1,0,0);
      break;
    case 27: 
      set(1,0,1,1,0,0);
      break;
    case 28: 
      set(0,1,1,1,0,0);
      break;



    default:
      set(1,1,1,0,0,0);
  }
}


void LedArray::set(const bool _r1, const bool _r2, const bool _r3, const bool _c1, const bool _c2, const bool _c3){
  digitalWrite(r1,_r1);
  digitalWrite(r2,_r2);
  digitalWrite(r3,_r3);
  digitalWrite(c1,_c1);
  digitalWrite(c2,_c2);
  digitalWrite(c3,_c3);
}
