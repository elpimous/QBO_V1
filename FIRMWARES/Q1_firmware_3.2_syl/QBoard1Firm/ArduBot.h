/*********************************************************************

   Software License Agreement (BSD License)

    Copyright (c) 2011, TheCorpora.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

 *   * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.
 *   * Neither the name of the TheCorpora nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.


   \author Miguel Angel Julian
 *********************************************************************/
#ifndef ArduBot_h
#define ArduBot_h

#include <inttypes.h>
#include <Arduino.h>
#include <avr/interrupt.h>
//---------------includes de las librerias usadas--------------//
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
//-----------------------     SRF10     -----------------------//

//-------------------------------------------------------------//
#include "comands.h"
#include "motores.h"
#include "LCDi2cW.h"
#include "SRF10.h"
//-------------------------------------------------------------//
#define MOTOR_1_CONTROL_1_PIN PE2
#define MOTOR_1_CONTROL_2_PIN PG3
#define MOTOR_2_CONTROL_2_PIN PH2
#define MOTOR_2_CONTROL_1_PIN PG4

#define MOTOR_1_COTROL_1_PORT PORTE
#define MOTOR_1_COTROL_2_PORT PORTG
#define MOTOR_2_COTROL_2_PORT PORTH
#define MOTOR_2_COTROL_1_PORT PORTG

#define MOTOR_1_COTROL_1_REGISTER DDRE
#define MOTOR_1_COTROL_2_REGISTER DDRG
#define MOTOR_2_COTROL_2_REGISTER DDRH
#define MOTOR_2_COTROL_1_REGISTER DDRG

#define MOTOR_1_PWM_ARDUINO_PIN 8 //PH5
#define MOTOR_2_PWM_ARDUINO_PIN 7 //PH4

#define MOTOR_1_HALL_A_PIN PJ2
#define MOTOR_1_HALL_B_PIN PJ3
#define MOTOR_2_HALL_A_PIN PJ4
#define MOTOR_2_HALL_B_PIN PJ5

#define MIN_FLOOR_DISTANCE_CM 11
#define MAX_FLOOR_DISTANCE_CM 35

#define FLOOR_DISTANCE_SENSOR_ARDUINO_PIN 8

#define BATTERY_LEVEL_INPUT_ARDUINO_PIN 9
#define BATTERY_LEVEL_DIGITAL_INPUT_ARDUINO_PIN 63

#define CHARGER_ARDUINO_PIN 22
//-------------------------------------------------------------//

namespace arduBot
{
//--------------------version de la libreria-------------------//
const uint8_t boardId = 0;
const uint8_t libraryVersion = 1;
//-------------------------------------------------------------//
//----------defines para la comunicacion con los SRF10---------//
const uint8_t srfCmdByte = 0x00;                         // Command byte
const uint8_t srfLightByte = 0x01;                       // Byte to read light sensor
const uint8_t srfRangeByte = 0x02;                       // Byte for start of ranging data
const uint8_t srfGainByte = 0x01;                       // Byte to read light sensor
//-------------------------------------------------------------//
class ArduBot
{
  private:
    byte alert_stop;
    //-------------------------------------------------------------//
    boolean isSrfUpdateContinuous;
    //-------------------------------------------------------------//
    static long spinLoopPeriodMs;
    //-------------------------------------------------------------//
    MPU6050 accelGyro;
    //-------------------------------------------------------------//
    double xCoordinate;
    double yCoordinate;
    double thetaCoordinate;
    void estimatePosition();
    void scanI2C();

    static bool wheelStopAlertFlag;
    static bool robotFallFlag;
    static bool robotCrashFlag;
  public:
    static bool robotStallFlag;
    static CMotors leftMotor;
    static CMotors rightMotor;
    static CBaseMovement par_motores;
    LCDi2cW lcd;
    boolean lcdState;
    boolean energyState;
    boolean accelGyroState;
    //-------------------------------------------------------------//
    //----variables para controlar los sensores de ultrasonido-----//
    byte NUM_SRFs;
    int SRFs_ADRESS[16];
    unsigned int SRFs_VALUES[16];
    byte NUM_SRFs_FRONT;
    int SRFs_FRONT[4];
    bool SRFs_FRONT_CRASH_FLAGS[16];
    byte NUM_SRFs_BACK;
    int SRFs_BACK[4];
    //-------------------------------------------------------------//
    //----variables para comprobar el sentido los motores----------//
    static bool leftMotorHallA;
    static bool leftMotorHallB;
    static bool rightMotorHallA;
    static bool rightMotorHallB;
    //-------------------------------------------------------------//
    //--------variables para de los sensores de infrarrojos--------//
    static byte ir1data;
    static byte ir2data;
    static byte ir3data;
    // ranging
    static uint16_t MiddleDist;
    
    //-------------------------------------------------------------//
    //------------------variables para el IMU----------------------//
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    int16_t accelerometerX;
    int16_t accelerometerY;
    int16_t accelerometerZ;
    int16_t roll;
    int16_t pitch; 

    int reading = 0;
    
    //-------------------------------------------------------------//
    //------------------------Constructor--------------------------//
    ArduBot(double wheelRadious = 0.102, double wheelDistance = 0.2736, int encoderResolution = 1560);
    //-------------------------------------------------------------//
    void begin(double spinLoopPeriodS = 0.005, double kp = 0.0, double ki = 0.0, double kd = 0.0);
    //-------------------------------------------------------------//
    //--------------Funcion para posicion y velocidad--------------//
    inline void getSpacePosition(double& x, double& y, double& angle)
    {
      x = xCoordinate;
      y = yCoordinate;
      angle = thetaCoordinate;
    };
    //----------------Funcion para calibrar el IMU---------------------//
    inline void calibrate() {
      //Change this 3 variables if you want to fine tune the skecth to your needs.
      int buffersize = 1000;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
      int acel_deadzone = 8;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
      int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
      int16_t ax, ay, az, gx, gy, gz;
      int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
      int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
      // reset offsets
      accelGyro.setXAccelOffset(0);
      accelGyro.setYAccelOffset(0);
      accelGyro.setZAccelOffset(0);
      accelGyro.setXGyroOffset(0);
      accelGyro.setYGyroOffset(0);
      accelGyro.setZGyroOffset(0);
      while (state != 2) {
        if (state == 0) {
          // first read the sensors several times to make an mean.
          long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
          while (i < (buffersize + 101)) {
            // read raw accel/gyro measurements from device
            accelGyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
            if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
              buff_ax = buff_ax + ax;
              buff_ay = buff_ay + ay;
              buff_az = buff_az + az;
              buff_gx = buff_gx + gx;
              buff_gy = buff_gy + gy;
              buff_gz = buff_gz + gz;
            }
            if (i == (buffersize + 100)) {
              mean_ax = buff_ax / buffersize;
              mean_ay = buff_ay / buffersize;
              mean_az = buff_az / buffersize;
              mean_gx = buff_gx / buffersize;
              mean_gy = buff_gy / buffersize;
              mean_gz = buff_gz / buffersize;
            }
            i++;
            delay(2); //Needed so we don't get repeated measures
          }
          state++;
          delay(10);
        }
    
        if (state == 1) {
          //compute the offsets to send to the MPU
          ax_offset = -mean_ax / 8;
          ay_offset = -mean_ay / 8;
          az_offset = (16384 - mean_az) / 8;
    
          gx_offset = -mean_gx / 4;
          gy_offset = -mean_gy / 4;
          gz_offset = -mean_gz / 4;
          accelGyro.setXAccelOffset(ax_offset);
          accelGyro.setYAccelOffset(ay_offset);
          accelGyro.setZAccelOffset(az_offset);
          accelGyro.setXGyroOffset(gx_offset);
          accelGyro.setYGyroOffset(gy_offset);
          accelGyro.setZGyroOffset(gz_offset);
          state++;
          delay(10);
        }
      }
    }
    inline void resetPosition()
    {
      xCoordinate = 0;
      yCoordinate = 0;
      thetaCoordinate = 0;
    };
    void setSpeeds(double linealSpeed, double angularSpeed);
    void updatePosition();
    //-------------------------------------------------------------//
    //----------Funcion para leer el sensor GP2D12-----------------//
    inline float getFloorDistance()
    {
      int analogValue = analogRead(FLOOR_DISTANCE_SENSOR_ARDUINO_PIN);
      float distance = 12343.85 * pow((float)analogValue, -1.15);
      if(distance<MIN_FLOOR_DISTANCE_CM || distance>MAX_FLOOR_DISTANCE_CM)
        ArduBot::wheelStopAlertFlag=true;
      else
        ArduBot::wheelStopAlertFlag=false;
      return distance;
    };
    inline unsigned int adcRead(byte pin)
    {
      return analogRead(pin);
    }
    //-------------------------------------------------------------//
    //---------------Funcion para leer los SRF10-------------------//
    int updateSrfRange(int address);
    unsigned int getSrfRange(int address);
    int getSrfLight(int address);
    int changeSrfAddress(int oldAddress, int newAddress);
    int changeSrfGain(int address, int gain);
    int changeSrfDistance(int address, int range);
    void testSrfs();
    void setSrfsRegisters();
    inline void setSrfContinuousUpdate(boolean value)
    {
      isSrfUpdateContinuous=value;
    }
    //-------------------------------------------------------------//
    //--------Funcion para leer el nivel de la bateria-------------//
    inline void getBatteryLevel(byte *value, byte *stat)
    {
      Wire.requestFrom(0x14, 2);
      *stat = Wire.read();
      *value = Wire.read();
      energyState = true;
    };
    //-------------------------------------------------------------//

    //-------------------------------------------------------------//
    //----------------Funcion para leer el IMU---------------------//
    void updateIMU();
    
    //-------------------------------------------------------------//
    inline void setK(byte k, float value)
    {
      switch (k)
      {
        case 0:
          //kp
          ArduBot::par_motores.kp = value;
          break;
        case 1:
          //ki
          ArduBot::par_motores.ki = value;
          break;
        case 2:
          //kd
          ArduBot::par_motores.kd = value;
          break;
      }
    }
    //-------------------------------------------------------------//
    void spinOnce();
};
}

#endif
