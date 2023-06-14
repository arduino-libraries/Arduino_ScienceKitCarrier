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

#ifndef __LED_ARRAY_H__
#define __LED_ARRAY_H__

#include "Arduino.h"

class LedArray{
  private:
    uint8_t r1, r2, r3, c1, c2, c3;
  public:
    LedArray(const uint8_t _r1, const uint8_t _r2, const uint8_t _r3, const uint8_t _c1, const uint8_t _c2, const uint8_t _c3);
    void set(const int x);
    void set(const bool _r1, const bool _r2, const bool _r3, const bool _c1, const bool _c2, const bool _c3);
};

#endif