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


#ifndef __PIN_DEF_H__
#define __PIN_DEF_H__

#include "Arduino.h"


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


//wave 1
#define FQ_1 A1     //p102
#define PH_1 A2     //p103

#define ROW_A_1 9  // D9  - p502
#define ROW_A_2 8  // D8  - p501
#define ROW_A_3 7  // D7  - p401
#define COL_A_1 6  // D6  - p400
#define COL_A_2 1  // D1  - p302
#define COL_A_3 27 // D27 - p206

#define DAC_W_1 19 // D19 - p013


//wave 2 freq selector  
#define FQ_2 A3    // A3 - p104
#define PH_2 A6    // A6 - p105

#define ROW_B_1 0  // D0
#define ROW_B_2 2  // D2
#define ROW_B_3 5  // D5
#define COL_B_1 4  // D4
#define COL_B_2 3  // D3
#define COL_B_3 24 // D24 - p109 // D10


//range selector
#define FQ_R    A0 // A0  - p101 
#define ROW_C_1 15 // D15 - p300
#define ROW_C_2 14 // D14 - p108
#define ROW_C_3 13 // D13 - p212
#define COL_C_1 23 // D23 p119 //11 // D11 - p214
#define COL_C_2 12 // D12 - p213






#endif