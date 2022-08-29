/*
  SRF10.cpp - Library for using SRF10 Rangefinders with Arduino
  Created by Senor Neil Dhar, April 11, 2014.
*/
#include <Arduino.h>
#include <Wire.h>
#include "SRF10.h"


SRF10::SRF10(TwoWire * _bus, byte Addr): i2cBus(_bus) {
	I2C_Address = Addr >> 1;
        USRange=70;
}

int SRF10::read(){
	i2cBus->beginTransmission(I2C_Address);
  	i2cBus->write(byte(0x02));
  	i2cBus->endTransmission();
	i2cBus->requestFrom(I2C_Address, 2);
	while(i2cBus->available() < 2); 
  	byte high = i2cBus->read();
  	byte low = i2cBus->read();
        temp=(high << 8) + low;
  	range = temp>0?temp:range;
  	return range;
}

int SRF10::asyncRead(byte units){
  if(millis()-lastRangeTime>USRange){
  	this->read();
     
	this->startRanging(units);
	lastRangeTime=millis();
  }
  return range;
}

void SRF10::startRanging(byte units){
  	//initiate a new ranging
	i2cBus->beginTransmission(I2C_Address);
	i2cBus->write(byte(0x00));
	i2cBus->write(byte(units));
	i2cBus->endTransmission();
}

int SRF10::setAddress(byte newAddr){
  i2cBus->beginTransmission(I2C_Address);
  i2cBus->write(byte(0x00));
  i2cBus->write(byte(0xA0));
  i2cBus->endTransmission();

  i2cBus->beginTransmission(I2C_Address);
  i2cBus->write(byte(0x00));
  i2cBus->write(byte(0xAA));
  i2cBus->endTransmission();

  i2cBus->beginTransmission(I2C_Address);
  i2cBus->write(byte(0x00));
  i2cBus->write(byte(0xA5));
  i2cBus->endTransmission();

  i2cBus->beginTransmission(I2C_Address);
  i2cBus->write(byte(0x00));
  i2cBus->write(newAddr);
  i2cBus->endTransmission();
  
  i2cBus->beginTransmission(newAddr);
  i2cBus->write(byte(0x00));  
  i2cBus->endTransmission();
  i2cBus->requestFrom(newAddr, 1);
  while(i2cBus->available() < 1);
  unsigned char test = i2cBus->read();
  
  if(test==5){
  	I2C_Address=newAddr;
  	return 1;
  }
  
  return 0;
}

void SRF10::setGain(int gain){
  i2cBus->beginTransmission(I2C_Address);
  i2cBus->write(byte(0x01));
  i2cBus->write(byte(gain));
  i2cBus->endTransmission();
}

void SRF10::setRange(double newRange){
  int rangeNo = (int)((double)((newRange-0.043)/0.043)+0.5);
  
  i2cBus->beginTransmission(I2C_Address);
  i2cBus->write(byte(0x02));
  i2cBus->write(rangeNo);
  i2cBus->endTransmission();
  USRange = (double)(0.28*rangeNo);
}
