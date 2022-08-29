/*
  SRF10.h - Library for using SRF10 Rangefinders with Arduino
  Created by Senor Neil Dhar, 11 April, 2014.
*/

#ifndef SRF10_h
#define SRF10_h

#include <Arduino.h>


  class SRF10{
    private: 
      TwoWire * i2cBus;
      byte I2C_Address;
      int range;
      int USRange;
      unsigned long long lastRangeTime;
      short temp;
    public:
      SRF10(TwoWire*, byte);
      int asyncRead(byte);
      int read();
      int setAddress(byte);
      void setGain(int);
      void setRange(double);
      void startRanging(byte);
  };

#endif
