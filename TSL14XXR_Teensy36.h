/*
  TSL14XXR_Teensy36.h
  Library for reading AMS TSL14XXR family linear photodiode array sensors
  and and IC Haus AMS TSL14XX compatable sensors, for use on Teensy 3.x boards

  (For Arduino 16 Mhz, use TSL14XXR_Arduino library instead!
  Unlike the Arduino version of this library, this version makes use of the
  dual ADCs of the Teensy 3.X boards. Teensy 3.X boards has a dual ADC where
  both analog input pins can be read simultaneously by the two ADCs, and we
  make use of this feature here.)

  Created by Douglas Mayhew,  March 14, 2017.

  MIT License

  Copyright (c) 2017 Douglas Mayhew

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/


#ifndef TSL14XXR_Teensy36_h
#define TSL14XXR_Teensy36_h

#include "ADC.h"   // Teensy ADC library
#include "Arduino.h"

class TSL14XXR_Teensy36
{
  public:
    TSL14XXR_Teensy36(int CLKpin, int SIpin, int Apin1, int Apin2);
    void init(uint16_t pixels, bool flushPixelsBeforeRead, uint16_t exposureMicroseconds);
    void readBytesSeries(uint8_t * data);
    void readBytesParallel(uint8_t * data);
    void readIntsSeries(uint16_t * data);
    void readIntsParallel(uint16_t * data);

  private:
    byte _CLKpin;
    byte _SIpin;
    byte _Apin1;
    byte _Apin2;

    bool _flushPixelsBeforeRead;
    uint16_t _pixels;
    uint16_t _halfPixels;
    uint16_t _exposureMicroseconds;
    uint16_t _sample1;
    uint16_t _sample2;
};

#endif
