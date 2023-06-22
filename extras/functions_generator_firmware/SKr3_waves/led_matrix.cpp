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

#include "led_matrix.h"

LedMatrix::LedMatrix(const uint8_t _r1, const uint8_t _r2, const uint8_t _r3, const uint8_t _c1, const uint8_t _c2){
  r1=_r1;
  r2=_r2;
  r3=_r3;
  c1=_c1;
  c2=_c2;

  pinMode(r1,OUTPUT);
  pinMode(r2,OUTPUT);
  pinMode(r3,OUTPUT);
  pinMode(c1,OUTPUT);
  pinMode(c2,OUTPUT);
}

/* frequency selector
   +-----+-----+--------+
   | ROW | COL |   LED  |
   +-----+-----+--------+
   | C_1 | C_1 |    1_1 |
   | C_2 | C_1 |   10_1 |
   | C_3 | C_1 |  100_1 |
   | C_1 | C_2 |    1_2 |
   | C_2 | C_2 |   10_2 |
   | C_3 | C_2 |  100_2 |   
   +-----+-----+--------+  
*/

void LedMatrix::set(const int x){
  switch(x){
    case 0: 
      set(0,1,1,1,0);
      break;
    case 1: 
      set(1,0,1,1,0);
      break;
    case 2: 
      set(1,1,0,1,0);  
      break;
    case 3: 
      set(0,1,1,0,1);
      break;
    case 4: 
      set(1,0,1,0,1);
      break;
    case 5: 
      set(1,1,0,0,1);
      break;
    default:
      set(1,1,1,0,0);
  }
}

void LedMatrix::set(const bool _r1, const bool _r2, const bool _r3, const bool _c1, const bool _c2){
  digitalWrite(r1,_r1);
  digitalWrite(r2,_r2);
  digitalWrite(r3,_r3);
  digitalWrite(c1,_c1);
  digitalWrite(c2,_c2);
}