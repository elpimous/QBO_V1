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

#include "ArduBot.h"

using namespace arduBot;
//---------Definicion de las variables static----------------------//
// states of encoders
bool ArduBot::leftMotorHallA = LOW;
bool ArduBot::leftMotorHallB = LOW;
bool ArduBot::rightMotorHallA = LOW;
bool ArduBot::rightMotorHallB = LOW;

// ranging
uint16_t ArduBot::LeftDist = 0;
uint16_t ArduBot::RightDist = 0;
uint16_t ArduBot::MiddleDist = 0;

// cycle times
long ArduBot::spinLoopPeriodMs = 5;
long spinTime = 0;
long srf10time = 0;
bool ArduBot::wheelStopAlertFlag=false;
bool ArduBot::robotFallFlag=false;
bool ArduBot::robotCrashFlag=false;
bool ArduBot::robotStallFlag=false;


// motors
//velocidad maxima del motor libre: 216rpm -> 22,619467109rad/s.
// nominal: 170-> 17,802358373rad/s
CMotors ArduBot::rightMotor = CMotors(&MOTOR_1_COTROL_1_REGISTER, &MOTOR_1_COTROL_2_REGISTER, &MOTOR_1_COTROL_1_PORT, &MOTOR_1_COTROL_2_PORT, MOTOR_1_CONTROL_1_PIN, MOTOR_1_CONTROL_2_PIN, MOTOR_1_PWM_ARDUINO_PIN);
CMotors ArduBot::leftMotor = CMotors(&MOTOR_2_COTROL_1_REGISTER, &MOTOR_2_COTROL_2_REGISTER, &MOTOR_2_COTROL_1_PORT, &MOTOR_2_COTROL_2_PORT, MOTOR_2_CONTROL_1_PIN, MOTOR_2_CONTROL_2_PIN, MOTOR_2_PWM_ARDUINO_PIN);
CBaseMovement ArduBot::par_motores = CBaseMovement(&ArduBot::leftMotor, &ArduBot::rightMotor);

//------------------------------------------------//
//--------Interrupcion para los motores-----------//
byte oldPort = 0;
ISR ( PCINT1_vect )
{
  byte port = PINJ & ((1 << MOTOR_1_HALL_A_PIN) | (1 << MOTOR_1_HALL_B_PIN) | (1 << MOTOR_2_HALL_A_PIN) | (1 << MOTOR_2_HALL_B_PIN));
  if (port == oldPort)
    return;
  oldPort = port;
  boolean newLeftMotorHallA = ((port & (1 << MOTOR_2_HALL_A_PIN)) != 0);
  boolean newLeftMotorHallB = ((port & (1 << MOTOR_2_HALL_B_PIN)) != 0);
  boolean newRightMotorHallA = ((port & (1 << MOTOR_1_HALL_A_PIN)) != 0);
  boolean newRightMotorHallB = ((port & (1 << MOTOR_1_HALL_B_PIN)) != 0);

  if (newLeftMotorHallA != ArduBot::leftMotorHallA)
  {
    if (newLeftMotorHallA == newLeftMotorHallB)
      ArduBot::leftMotor.pulsesDifference--;
    else
      ArduBot::leftMotor.pulsesDifference++;
    ArduBot::leftMotorHallA = newLeftMotorHallA;
  }
  else if (newLeftMotorHallB != ArduBot::leftMotorHallB)
  {
    if (newLeftMotorHallB != newLeftMotorHallA)
      ArduBot::leftMotor.pulsesDifference--;
    else
      ArduBot::leftMotor.pulsesDifference++;
    ArduBot::leftMotorHallB = newLeftMotorHallB;
  }

  if (newRightMotorHallA != ArduBot::rightMotorHallA)
  {
    if (newRightMotorHallA == newRightMotorHallB)
      ArduBot::rightMotor.pulsesDifference++;
    else
      ArduBot::rightMotor.pulsesDifference--;
    ArduBot::rightMotorHallA = newRightMotorHallA;
  }
  else if (newRightMotorHallB != ArduBot::rightMotorHallB)
  {
    if (newRightMotorHallB != newRightMotorHallA)
      ArduBot::rightMotor.pulsesDifference++;
    else
      ArduBot::rightMotor.pulsesDifference--;
    ArduBot::rightMotorHallB = newRightMotorHallB;
  }
}
//----------------------------------------------------------//
//----------Inicializacion del sistema----------------------//
ArduBot::ArduBot(double wheelRadius, double wheelDistance, int encoderResolution) : lcd(4, 20, 0x63, 0), lcdState(false), energyState(false), accelGyro(0x68), accelGyroState(false)
{
  //----------Configuracion de los pines---------//
  DDRJ &= ~(1 << MOTOR_1_HALL_A_PIN) & ~(1 << MOTOR_1_HALL_B_PIN) & ~(1 << MOTOR_2_HALL_A_PIN) & ~(1 << MOTOR_2_HALL_B_PIN);  //Pines de los sensores HALL como entradas
  PORTJ |= (1 << MOTOR_1_HALL_A_PIN) | (1 << MOTOR_1_HALL_B_PIN) | (1 << MOTOR_2_HALL_A_PIN) | (1 << MOTOR_2_HALL_B_PIN);  //Resistencias de pull-up para sensores hall
  //---------------------------------------------//
  //-----Interrupciones para los motores---------//
  PCICR |= (1 << PCIE1);  //Habilito las interrupciones de los motores
  //---------------------------------------------//
  //-----Interrupciones para los IR--------------//
  DDRE &= ~(1 << 6) & ~(1 << 7);  //Pines de los sensores IR como entradas
  DDRD &= ~(1 << 2);  //Pines de los sensores IR como entradas
  PORTE |= (1 << 6) | (1 << 7);  //Resistencias de pull-up para IRs
  PORTD |= (1 << 2);  //Resistencias de pull-up para IRs
  EICRA &= ~(1 << ISC20) & ~(1 << ISC30);  //flanco de bajada (IR3 y Lect_Pulsador)
  EICRA |= (1 << ISC21) | (1 << ISC31);  //flanco de bajada (IR3 y Lect_Pulsador)
  EICRB &= ~(1 << ISC60) & ~(1 << ISC70);  //flanco de bajada (IR1 e IR2)
  EICRB |= (1 << ISC61) | (1 << ISC71);  //flanco de bajada (IR1 e IR2)
  //---------------------------------------------//
  ArduBot::par_motores.initializePhisicalVariables(encoderResolution, wheelDistance, wheelRadius);
  xCoordinate = 0;
  yCoordinate = 0;
  thetaCoordinate = 0;
  alert_stop = 0;
  //---------------------------------------------//
}

void ArduBot::begin(double spinLoopPeriodS, double kp, double ki, double kd)
{
  DDRB = DDRB | B10001100;
  PORTB = 0;
  //-----Inicializacion del puerto serie---------//
  Serial.begin(115200);
  Serial.flush();
  //-------Inicializacion del puerto I2C---------//
  Wire.begin();
  Wire.setClock(400000);
  //-------Inicializacion del LCD----------------//
  lcd.init();
  lcd.clear();
  lcd.home();
  delay(100);
  lcd.print("Calibrating...");
  //-------Inicializacion del MPU6050----------------//
  accelGyro.initialize();
  accelGyroState = accelGyro.testConnection();
  calibrate();
  delay(100);
  lcd.clear();
  lcd.home();
  lcd.print("Calibrated!");

  //-------Reading energy status---------------------//
  byte stat = 0;
  byte value = 0;
  energyState = true;
  getBatteryLevel(&value, &stat);
  //-------Init audio ampli---------------------//
  stat >>= 1;
  stat &= 0x01;
  if (stat == 1)
  {
    //Ampli a volumen moderado para que tarde mas en romperse
    Wire.beginTransmission(B1001011);
    Wire.write(37);                             // Send Command Byte
    Wire.endTransmission();
  }
  else
  {
    //Ampli a mute por estar el PC apagado
    Wire.beginTransmission(B1001011);
    Wire.write(0);                             // Send Command Byte
    Wire.endTransmission();
  }

  oldPort = PINJ & ((1 << MOTOR_1_HALL_A_PIN) | (1 << MOTOR_1_HALL_B_PIN) | (1 << MOTOR_2_HALL_A_PIN) | (1 << MOTOR_2_HALL_B_PIN));
  leftMotorHallA = ((oldPort & (1 << MOTOR_1_HALL_A_PIN)) != 0);
  leftMotorHallB = ((oldPort & (1 << MOTOR_1_HALL_B_PIN)) != 0);
  rightMotorHallA = ((oldPort & (1 << MOTOR_2_HALL_A_PIN)) != 0);
  rightMotorHallB = ((oldPort & (1 << MOTOR_2_HALL_B_PIN)) != 0);
  ArduBot::spinLoopPeriodMs = long(spinLoopPeriodS * 1000);
  ArduBot::par_motores.initializePIDVariables(kp, kd, ki, spinLoopPeriodS);
  spinTime = srf10time = millis();
  TCCR4B = TCCR4B & 0b11111000 | 0x01; //Frecuencia del pwm al máximo para evitar el ruido en los motores (otra )
  PCMSK1 = 0 | (1 << PCINT11) | (1 << PCINT12) | (1 << PCINT13) | (1 << PCINT14);  //Pines que interrumpen para los encoders de los motores
  EIMSK |= (1 << INT2) | (1 << INT6) | (1 << INT7) | (1 << INT3); //Las pongo en marcha
}
//----------------------------------------------------------//
//-----------------Funciones para los encoders--------------//


long old_time = millis();
long stopTime = 1000;
long speedComandTime = 0;
void ArduBot::updatePosition()
{
  long now = millis();
  if (now > speedComandTime + stopTime)
  {
    setSpeeds(0, 0);
  }
  ArduBot::par_motores.doControlLoop();
  estimatePosition();
}


void ArduBot::estimatePosition()
{
  float odometryLinearMovement = 1.05*ArduBot::par_motores.actualLinearMovement; // 1.05
  float odometryAngularMovement = 1.0256*ArduBot::par_motores.actualAngularMovement; // (360/351  -> 1.0256)
  float gyroAngularMovement = ((float)gyroZ) * 0.000133158 * 0.005;
  float difference = abs(odometryAngularMovement - gyroAngularMovement);
  float angularMovement = 0.0;
  float treshold = 0.0001;//rad
  if(abs(gyroAngularMovement)>treshold){
    angularMovement = gyroAngularMovement;
  }
  float x = odometryLinearMovement * cos(thetaCoordinate + angularMovement / 2.0);
  float y = odometryLinearMovement * sin(thetaCoordinate + angularMovement / 2.0);

  xCoordinate += x;
  yCoordinate += y;
  thetaCoordinate += angularMovement;
};

void ArduBot::scanI2C() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
}

// polar control of speed linear_speed in m/s and angular speed in rad/s
void ArduBot::setSpeeds(double linear_speed, double angular_speed)
{
  lcd.setCursor(1, 0);
  lcd.print("                    ");
  if(MiddleDist<MIN_FLOOR_DISTANCE_CM || MiddleDist>MAX_FLOOR_DISTANCE_CM){
    lcd.setCursor(1, 0);
    lcd.print("Alert : Stop");
    if(linear_speed>0){
      linear_speed = 0.0;
    }
  }
  
  if(abs(roll)>30 || abs(pitch)>30){
    lcd.setCursor(1, 0);
    lcd.print("Alert : Fall");
    linear_speed = 0.0;
    angular_speed = 0.0;
  }

  if(LeftDist<20 && LeftDist!=0){
    lcd.setCursor(1, 0);
    lcd.print("Alert : Crash");
    if(linear_speed>0){
      linear_speed = 0.0;
    }
  }

  if(RightDist<20 && RightDist!=0){
    lcd.setCursor(1, 0);
    lcd.print("Alert : Crash");
    if(linear_speed>0){
      linear_speed = 0.0;
    }
  }
  
  
  ArduBot::par_motores.desiredLinearSpeed = linear_speed;
  ArduBot::par_motores.desiredAngularSpeed = angular_speed;
  ArduBot::par_motores.transformSpeeds2Pulses();
  speedComandTime = millis();
}

//----------------------------------------------------------//
//boolean front=true;
byte index_front = 0;
byte index_back = 0;
long accelerometerTime = 0;
long batteryTime = 0;
byte accelerometer_fall_count = 0;

void ArduBot::spinOnce() {
  //Calculo de la posicion de l robot y procesamiento del PID
  long now = millis();
  if (now - spinTime > ArduBot::spinLoopPeriodMs)
  {
    spinTime += ArduBot::spinLoopPeriodMs;
    accelGyro.getMotion6(&accelerometerX, &accelerometerY, &accelerometerZ, &gyroX, &gyroY, &gyroZ);
    accelGyro.setDLPFMode(2); // +- 1000°/s
    roll = 180.0*accelerometerY/(accelerometerZ*3.1416);
    pitch = 180.0*accelerometerX/(accelerometerZ*3.1416);
    ArduBot::updatePosition();// give orders to motors
  }
  if (now - srf10time > 130) //65
  {
    srf10time += 130;
    MiddleDist = getFloorDistance();

    lcd.setCursor(3, 0);
    lcd.print("                    ");
    lcd.setCursor(3, 0);
    lcd.print((int)(xCoordinate*100));
    lcd.setCursor(3, 5);
    lcd.print((int)(thetaCoordinate*180.0/PI));
    lcd.setCursor(3, 10);
    lcd.print(max(abs(pitch),abs(roll)));
    lcd.setCursor(3, 16);
    lcd.print((int)(yCoordinate*100));
  }
  if (now - batteryTime > 10000)
  {
    batteryTime += 10000;
    byte stat = 0;
    byte value = 0;
    energyState = true;
    getBatteryLevel(&value, &stat);
    lcd.setCursor(2, 0);
    lcd.print("                    ");
    lcd.setCursor(2, 0);
    lcd.print("Bat: ");
    float batV = ((float)value / 10);
    lcd.print(batV, 1);
    lcd.print("V");
    lcd.setCursor(2, 12);
    lcd.print("Stat: ");
    lcd.print(stat, DEC);
    stat >>= 1;
    stat &= 0x01;
    if (stat == 1)
    {
      //Ampli a volumen moderado para que tarde mas en romperse
      Wire.beginTransmission(B1001011);
      Wire.write(37);                             // Send Command Byte
      Wire.endTransmission();
    }
    else
    {
      //Ampli a mute por estar el PC apagado
      Wire.beginTransmission(B1001011);
      Wire.write(0);                             // Send Command Byte
      Wire.endTransmission();
    }
  }
}
