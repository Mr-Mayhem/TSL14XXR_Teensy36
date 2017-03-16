/*
  TSL14XXR_Teensy36.cpp
  Library for reading AMS TSL14XXR family linear photodiode array sensors
  and and IC Haus AMS TSL14XX compatable sensors, for use on Teensy 3.x boards

  (For Arduino 16 Mhz, use TSL14XXR_Arduino library instead!
  Unlike the Arduino version of this library, this version makes use of the
  dual ADCs of the Teensy 3.X boards. Teensy 3.X boards has a dual ADC where
  both analog input pins can be read simultaneously by the two ADCs, which
  we take advantage of in this version.

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

#include "Arduino.h"
#include "ADC.h" // Teensy ADC library
#include "TSL14XXR_Teensy36.h"
// ============================================================================================================
// Teensy 3.X Pins for the TSL1402R sensor: (right side comments show sensor connections) =====================
// (Note all AMS and IC Haus linear array sensors of this family use similiar inputs and
// outputs and work with this library. Refer to the data sheet for your particular model
// of sensor for sensors not shown below)
byte _CLKpin = 24; // <-- Teensy pin delivering the clock pulses to pin 3(CLK) of the TSL1402R
byte _SIpin = 25;  // <-- Teensy pin delivering the SI (serial-input) pulse to pin 2 of the TSL1402R
byte _Apin1 = 14;  // <-- Teensy pin connected to pin 4 (analog output 1) of the TSL1402R
byte _Apin2 = 39;  // <-- Teensy pin connected to pin 8 (analog output 2) (parallel mode)

// Teensy 3.X Pins for the TSL1410R sensor: ==================================================================
// byte _CLKpin = 24;  // <-- Teensy pin delivering the clock pulses to pin 4(CLK) of the TSL1410R
// byte _SIpin =  25;  // <-- Teensy pin delivering the SI (serial-input) pulse to pin 2 of the TSL1410R
// byte _Apin1 =  14;  // <-- Teensy pin connected to pin 6 (analog output 1) of the TSL1410R
// byte _Apin2 =  39;  // <-- Teensy pin connected to pin 12 (analog output 2) (parallel mode)

// ===========================================================================================================

// (flushSensorPixelsBeforeRead Notes):
// True = Clock the pixels out, and re-read them again before exiting, remember to add exposureMicroseconds
// False = Read the pixel values as they are, which have been collecting light since the previous pass.
// Not flushing is about twice as fast, but is not usable with long delays between reads of the sensor, 
// such as drawing to the screen each frame (I do every 100 to avoid this), long calculations, etc., 
// because the photodiodes saturate while waiting to be read out on the following read cycle.
// Setting to False also makes occasional glitches in the ADC values, because occasionally Teensy 3.X returns 
// to the next sensor read after a significantly larger delay (for performing housekeeping, who knows),
// resulting in a spike of the values. Flushing the pixels by setting flushSensorPixelsBeforeRead = true, 
// or turning up ADC averaging to 2 (on both ADC settings in the main sketch) resolves the glitches, 
// but at the cost of half the speed.
bool _flushPixelsBeforeRead;

uint16_t _sample1; // temporary place to hold A0 ADC value
uint16_t _sample2; // temporary place to hold A1 ADC value
uint16_t _pixels;  // total number of pixels in the sensor, must be an even number
uint16_t _halfPixels; // half the total pixels

// (exposureMicroseconds Notes): is the number of extra microseconds to allow light to accumulate on all
// the photodiodes.
// Adding delay results in a higher signal level for all pixels.
// If set too low, the sensor may deliver low signals, and set it too high, and it saturates the signal.
// If you do not flush pixels (flushPixelsBeforeRead = false), set exposureMicroseconds to 0,
// because the signal is probably high already, and adding delay makes it likely to saturate hard
// at max signal level for all pixels. If set to zero, we skip the delayMicroseconds() function
// entirely, which might help reduce jitter by not yielding the thread.
uint16_t _exposureMicroseconds;

// The ADC library object
extern ADC *adc;

// Tells ADC library to read both ADCs at the same instant in Teensy 3.X
extern ADC::Sync_result ADCresult;

TSL14XXR_Teensy36::TSL14XXR_Teensy36(int CLKpin, int SIpin, int Apin1, int Apin2)
{
  _CLKpin = CLKpin;
  _SIpin = SIpin;
  _Apin1 = Apin1;
  _Apin2 = Apin2;
}

// init ======================================================================================================
void TSL14XXR_Teensy36::init(uint16_t pixels, bool flushPixelsBeforeRead, uint16_t exposureMicroseconds)
{
  // Initialize Teensy pins
  pinMode(_CLKpin, OUTPUT);
  pinMode(_SIpin, OUTPUT);

  pinMode(_Apin1, INPUT);
  pinMode(_Apin2, INPUT);

  _pixels = pixels;
  _halfPixels = _pixels >> 1;
  _flushPixelsBeforeRead = flushPixelsBeforeRead;
  _exposureMicroseconds = exposureMicroseconds;

  // Set output pins low:
  digitalWriteFast(_SIpin, LOW);
  digitalWriteFast(_CLKpin, LOW);

  // Clock out any existing SI pulse through the ccd register:
  for (int i = 0; i < _pixels + 2; i++) { // for each pixel
    digitalWriteFast(_CLKpin, HIGH);
    digitalWriteFast(_CLKpin, LOW);
  }

  // Create a new SI pulse...
  digitalWriteFast(_SIpin, HIGH);
  digitalWriteFast(_CLKpin, HIGH);
  digitalWriteFast(_SIpin, LOW);
  digitalWriteFast(_CLKpin, LOW);

  // ... and clock out that same SI pulse through the sensor register:
  for (int i = 0; i < _pixels + 2; i++)
  {
    digitalWriteFast(_CLKpin, HIGH);
    digitalWriteFast(_CLKpin, LOW);
  }
}
// end init ==================================================================================================

void TSL14XXR_Teensy36::readBytesSeries(uint8_t * data)
{
  if (_flushPixelsBeforeRead)
  {
    // Create a new SI pulse:
    digitalWrite(_SIpin, HIGH);
    digitalWrite(_CLKpin, HIGH);
    digitalWrite(_SIpin, LOW);
    digitalWrite(_CLKpin, LOW);

    // flush the pixels
    for (int i = 0; i < _pixels; i ++)
    {
      digitalWrite(_CLKpin, HIGH);
      digitalWrite(_CLKpin, LOW);
    }
  }

  if (_exposureMicroseconds > 0)
  {
    // This adds a delay to allow the photodiodes to gather more light, resulting in higher signal
    // for all pixels. If set to low, the sensor may deliver low signals, set too high, and it
    // saturates the signal. If you do not flush pixels, set it to 0, because the signal is
    // probably high already, and adding delay makes it likely to saturate hard at max signal level
    // for all pixels.
    // Add _exposureMicroseconds integration time
    delayMicroseconds(_exposureMicroseconds);
  }

  // Create a new SI pulse:
  digitalWrite(_SIpin, HIGH);
  digitalWrite(_CLKpin, HIGH);
  digitalWrite(_SIpin, LOW);
  digitalWrite(_CLKpin, LOW);

  // read the pixels (series mode)
  for (int i = 0; i < _pixels; i++)
  {
    // read the pixel (we read both pins like parallel but ignore the Apin2 value)
    _sample1 = adc->analogRead(_Apin1);

    // send one clock pulse to the sensor
    digitalWrite(_CLKpin, HIGH);
    digitalWrite(_CLKpin, LOW);

    // write two uint_8 (bytes) to data[] for each pixel
    data[i << 1] = (_sample1 >> 8) & 0xFF;               // AOpin1 HighByte
    data[(i << 1) + 1] = _sample1 & 0xFF;                 // AOpin1 LowByte
  }
  // the sensor data is now in the array that was passed into this function.

}

void TSL14XXR_Teensy36::readBytesParallel(uint8_t * data)
{
  if (_flushPixelsBeforeRead)
  {
    // Create a new SI pulse:
    digitalWrite(_SIpin, HIGH);
    digitalWrite(_CLKpin, HIGH);
    digitalWrite(_SIpin, LOW);
    digitalWrite(_CLKpin, LOW);

    // flush the pixels
    for (int i = 0; i < _pixels; i ++)
    {
      digitalWrite(_CLKpin, HIGH);
      digitalWrite(_CLKpin, LOW);
    }
  }

  if (_exposureMicroseconds > 0)
  {
    // Add _exposureMicroseconds integration time
    delayMicroseconds(_exposureMicroseconds);
  }

  // Create a new SI pulse:
  digitalWrite(_SIpin, HIGH);
  digitalWrite(_CLKpin, HIGH);
  digitalWrite(_SIpin, LOW);
  digitalWrite(_CLKpin, LOW);

  // read the pixels (parallel mode)
  for (int i = 0; i < _halfPixels; i++)
  {
    // read 2 pixels (we read both pins at the same instant using Teensy ADC library)
    ADCresult = adc->analogSynchronizedRead(_Apin1, _Apin2);

    // send one clock pulse to the sensor
    digitalWrite(_CLKpin, HIGH);
    digitalWrite(_CLKpin, LOW);

    // copy the 2 pixel values to simple variables
    _sample1 = ADCresult.result_adc0; // ADC sample 1
    _sample2 = ADCresult.result_adc1; // ADC sample 2

    // for each of the 2 pixel values, we write two uint_8 (bytes) to data[]
    // Pixel 1:
    data[i << 1] = (_sample1 >> 8) & 0xFF; // AOpin1 HighByte
    data[(i << 1) + 1] = _sample1 & 0xFF;  // AOpin1 LowByte

    // Pixel 2:
    data[(i + _halfPixels) << 1] = (_sample2 >> 8) & 0xFF; // AOpin2 HighByte
    data[((i + _halfPixels) << 1) + 1] = _sample2 & 0xFF;  // AOpin2 LowByte
  }

  // the sensor data is now in the array that was passed into this function.

}

void TSL14XXR_Teensy36::readIntsSeries(uint16_t * data)
{
  if (_flushPixelsBeforeRead)
  {
    // Create a new SI pulse:
    digitalWrite(_SIpin, HIGH);
    digitalWrite(_CLKpin, HIGH);
    digitalWrite(_SIpin, LOW);
    digitalWrite(_CLKpin, LOW);

    // flush the pixels
    for (int i = 0; i < _pixels; i ++)
    {
      digitalWrite(_CLKpin, HIGH);
      digitalWrite(_CLKpin, LOW);
    }
  }

  if (_exposureMicroseconds > 0)
  {
    // Add _exposureMicroseconds integration time
    delayMicroseconds(_exposureMicroseconds);
  }

  // Create a new SI pulse:
  digitalWrite(_SIpin, HIGH);
  digitalWrite(_CLKpin, HIGH);
  digitalWrite(_SIpin, LOW);
  digitalWrite(_CLKpin, LOW);


  // read the pixels (series mode)
  for (int i = 0; i < _pixels; i++)
  {
    // read the pixel
    _sample1 = adc->analogRead(_Apin1);

    // send one clock pulse to the sensor
    digitalWrite(_CLKpin, HIGH);
    digitalWrite(_CLKpin, LOW);

    // for each pixel value, we write one uint_16 (short integer) to data[]
    data[i] = _sample1; // each pixel
  }

  // the sensor data is now in the array that was passed into this function.

}

void TSL14XXR_Teensy36::readIntsParallel(uint16_t * data)
{
  if (_flushPixelsBeforeRead)
  {
    // Create a new SI pulse:
    digitalWriteFast(_SIpin, HIGH);
    digitalWriteFast(_CLKpin, HIGH);
    digitalWriteFast(_SIpin, LOW);
    digitalWriteFast(_CLKpin, LOW);

    // flush the pixels
    for (int i = 0; i < _pixels; i ++)
    {
      digitalWriteFast(_CLKpin, HIGH);
      digitalWriteFast(_CLKpin, LOW);
    }
  }

  if (_exposureMicroseconds > 0)
  {
    // Add _exposureMicroseconds integration time
    delayMicroseconds(_exposureMicroseconds);
  }

  // Create a new SI pulse:
  digitalWriteFast(_SIpin, HIGH);
  digitalWriteFast(_CLKpin, HIGH);
  digitalWriteFast(_SIpin, LOW);
  digitalWriteFast(_CLKpin, LOW);


  // read the pixels (parallel mode)
  for (int i = 0; i < _halfPixels; i++)
  {
    // read 2 pixels (we read both pins at the same instant using Teensy ADC library)
    ADCresult = adc->analogSynchronizedRead(_Apin1, _Apin2);

    // send one clock pulse to the sensor
    digitalWriteFast(_CLKpin, HIGH);
    digitalWriteFast(_CLKpin, LOW);

    // for each of the 2 pixel values, we write two uint_16 (short integers) to data[]
    // Pixel 1:
    data[i] = ADCresult.result_adc0;

    // Pixel 2:
    data[i + _halfPixels] = ADCresult.result_adc1;
  }

  // the sensor data is now in the array that was passed into this function.

}


