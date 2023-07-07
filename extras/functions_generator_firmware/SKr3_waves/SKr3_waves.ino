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

#include "Wire.h"
#include "pin_def.h"
#include "led_gauge.h"
#include "led_range.h"
#include "analogWave.h"

#define ANRES 9

#define RES 100

// Firmware version
uint8_t version[2]={0,6};

char command=0;
byte data[6];

uint16_t wave[RES];

LedGauge frequency_1(FQ_1, 10, 10, ROW_A_1, ROW_A_2, ROW_A_3, COL_A_1, COL_A_2, COL_A_3);
LedGauge frequency_2(FQ_2, 10, 10, ROW_B_1, ROW_B_2, ROW_B_3, COL_B_1, COL_B_2, COL_B_3, true);

LedRange range(FQ_R, ROW_C_1, ROW_C_2, ROW_C_3, COL_C_1, COL_C_2);

analogWave wave1(DAC,wave,RES,0);
analogWave wave2(DAC2,wave,RES,0);

int previous_frequency_1 = 10;
int previous_frequency_2 = 10;
int freq1 = 10;
int freq2 = 10;

int previous_phase_1 = 0;
int previous_phase_2 = 0;

int phase1 = 0;
int phase2 = 0;

const float step = ADC_RES/float(RES);
const float phase_to_deg = 180/float(RES);

bool test_flag = false;
unsigned long time_rephase;
int rephase;



// This function loads an wave array with a sinewave using RES samples.

void generate_wave(){
  float s = (2 * 3.1415) / RES;
  for (auto i = 0; i < RES; ++i) {
    float sample = (1 + sin((float)i * s)) * (float)65535 / 2.0;
    wave[i] = (uint16_t)sample;
  }
}


void comm_event(int event){
  command=Wire.read();
}

void comm_request(){
  switch(command){
    case 'V':                         // version, high rev, low rev
      Wire.write(version,2);
      break;
    case 'D':
      data[0]=frequency_1.getGaugeValue();
      data[1]=range.getRange1();
      data[2]=phase1*phase_to_deg;
      data[3]=frequency_2.getGaugeValue();
      data[4]=range.getRange2();
      data[5]=phase2*phase_to_deg;
      Wire.write(data,6);
      break;
    /*
    case 'T':
      test_flag=true;
      break;
    case 'N':
      test_flag=false;
      break;
    */
  }
}


void setup() {
  Serial.begin(115200);
  generate_wave();
  pinMode(PH_1,INPUT);
  pinMode(PH_2,INPUT);
  frequency_1.refresh();
  frequency_2.refresh();
  range.update();

  Wire.begin(0x2F);
  Wire.onReceive(comm_event);
  Wire.onRequest(comm_request);

  //wave1.sine(freq1); //these use default sampling (24)
  //wave2.sine(freq1);
  wave1.begin(freq1);
  wave2.begin(freq2);
  wave1.sync(wave2);
  rephase=0;
  time_rephase=millis();

  /* 20230707 keep resolution to 0-511 also if core now is correct
    and goes from 1-1023  */
  analogReadResolution(ANRES);
}



void loop() {
  frequency_1.refresh();
  frequency_2.refresh();
  range.update();

  freq1=frequency_1.getGaugeValue()*range.getRange1();
  freq2=frequency_2.getGaugeValue()*range.getRange2();

  phase1=analogRead(PH_1)/step;
  if (phase1>(RES+10)){
    phase1=0;
  }

  phase2=analogRead(PH_2)/step;
  if (phase2>RES){
    phase2=0;
  }

  if (freq1!=previous_frequency_1){
    wave1.freq(freq1);
    previous_frequency_1=freq1;
    wave1.sync(wave2);
  }

  if (freq2!=previous_frequency_2){
    wave2.freq(freq2);
    previous_frequency_2=freq2;
    wave2.sync(wave1);
  }

  if (millis()-time_rephase>500){
    if (phase1!=previous_phase_1){
      wave1.offset(phase1/2);
      previous_phase_1=phase1;
      if (phase2==0){
        wave1.sync(wave2);
      }
    }
    if (phase2!=previous_phase_2){
      wave2.offset(phase2/2);
      previous_phase_2=phase2;
      if (phase1==0){
        wave2.sync(wave1);
      }
    }
    time_rephase=millis();
  }




  delay(1);
}
