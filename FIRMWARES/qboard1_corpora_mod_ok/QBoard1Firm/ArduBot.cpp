/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, TheCorpora.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TheCorpora nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.

 *
 * \author Miguel Angel Julian
 *********************************************************************/
 
#include "ArduBot.h"
#include <avr/interrupt.h>

//#define WHEEL_STOP_ALERT
//#define ROBOT_FALL_ALERT
//#define ROBOT_CRASH_ALERT
//#define ROBOT_STALL_ALERT

using namespace arduBot;
//---------Definicion de las variables static----------------------//
bool ArduBot::leftMotorHallA=LOW;
bool ArduBot::leftMotorHallB=LOW;
bool ArduBot::rightMotorHallA=LOW;
bool ArduBot::rightMotorHallB=LOW;

byte ArduBot::ir1data=0;
byte ArduBot::ir2data=0;
byte ArduBot::ir3data=0;

long ArduBot::spinLoopPeriodMs=5;
long spinTime=0;
long srf10time=0;
bool ArduBot::wheelStopAlertFlag=false;
bool ArduBot::robotFallFlag=false;
bool ArduBot::robotCrashFlag=false;
bool ArduBot::robotStallFlag=false;
//velocidad maxima del motor libre: 216rpm -> 22,619467109rad/s. nominal: 170-> 17,802358373rad/s
//double ArduBot::max_accel=0.1;
CMotors ArduBot::rightMotor=CMotors(&MOTOR_1_COTROL_1_REGISTER, &MOTOR_1_COTROL_2_REGISTER, &MOTOR_1_COTROL_1_PORT, &MOTOR_1_COTROL_2_PORT, MOTOR_1_CONTROL_1_PIN, MOTOR_1_CONTROL_2_PIN, MOTOR_1_PWM_ARDUINO_PIN);
CMotors ArduBot::leftMotor=CMotors(&MOTOR_2_COTROL_1_REGISTER, &MOTOR_2_COTROL_2_REGISTER, &MOTOR_2_COTROL_1_PORT, &MOTOR_2_COTROL_2_PORT, MOTOR_2_CONTROL_1_PIN, MOTOR_2_CONTROL_2_PIN, MOTOR_2_PWM_ARDUINO_PIN);
CBaseMovement ArduBot::par_motores=CBaseMovement(&ArduBot::leftMotor,&ArduBot::rightMotor);


//------------------------------------------------//
//--------Interrupciones para los IR--------------//
byte capturedDataHi[3]={0,0,0};
byte capturedDataLo[3]={0xFF,0xFF,0xFF};
byte capturedBits=0;

long microsec=0;
boolean start_bit[3]={false,false,false};

void readIRByte()
{
  byte readedBit[3];
  
  readedBit[0]=(PINE&(1<<PE6))!=0 ? 1 : 0;
  readedBit[1]=(PINE&(1<<PE7))!=0 ? 1 : 0;
  readedBit[2]=(PIND&(1<<PD2))!=0 ? 1 : 0;
  capturedBits++;
  for(int i=0;i<3;i++)
  {
    if(capturedBits==1&&readedBit[i]==0)
    {
      start_bit[i]=true;
      capturedDataHi[i]=0;
      capturedDataLo[i]=0xFF;
    }
    else if(capturedBits==1&&readedBit[i]==1)
    {
      start_bit[i]=false;
      capturedDataHi[i]=0;
      capturedDataLo[i]=0xFF;
    }
    else if((capturedBits>1)&&(capturedBits<10)&&(start_bit[i]==true))
    {
      if(capturedBits<6)
        capturedDataHi[i] = (capturedDataHi[i]<<1) | readedBit[i];
      else
        capturedDataLo[i] = (capturedDataLo[i]<<1) | readedBit[i];
    }
    else if((capturedBits==10)&&(start_bit[i]==true))
    {
      if(readedBit[i]==1)
      {
        if((byte(capturedDataHi[i])==(byte(~capturedDataLo[i]))))
        {
          switch(i)
          {
            case 0:
              //Serial.print("IR1: ");
              ArduBot::ir1data=capturedDataLo[i];
              break;
            case 1:
              //Serial.print("IR2: ");
              ArduBot::ir2data=capturedDataLo[i];
              break;
            case 2:
              //Serial.print("IR3: ");
              ArduBot::ir3data=capturedDataLo[i];
              break;
            default:
              break;
          }
          //Serial.println(capturedDataLo[i],BIN);
        }
        capturedDataHi[i]=0;
        capturedDataLo[i]=0xFF;
        start_bit[i]=false;
      }
      IRbeanSerial::stop();
      EIMSK |= (1 << INT2) | (1 << INT6) | (1 << INT7); //Las pongo en marcha
    }
    else if(capturedBits>=10)
    {
      if(readedBit[i]==1)
      {
        capturedDataHi[i]=0;
        capturedDataLo[i]=0xFF;
        start_bit[i]=false;
      }
      IRbeanSerial::stop();
      EIMSK |= (1 << INT2) | (1 << INT6) | (1 << INT7); //Las pongo en marcha
    }
  }
  if(((start_bit[0]==false)&&(start_bit[1]==false)&&(start_bit[2]==false)))
  {
    capturedBits=0;
    IRbeanSerial::stop();
    EIMSK |= (1 << INT2) | (1 << INT6) | (1 << INT7); //Las pongo en marcha
  }
}

ISR (INT2_vect)  //Interrupcion para el IR3
{
  EIMSK &= ~(1 << INT2) & ~(1 << INT6) & ~(1 << INT7); //Las quito
  capturedBits=0;
  //ArduBot::ir3data=0;
  IRbeanSerial::start();
}

ISR (INT6_vect)  //Interrupcion para el IR1
{
  EIMSK &= ~(1 << INT2) & ~(1 << INT6) & ~(1 << INT7); //Las quito
  capturedBits=0;
  //ArduBot::ir1data=0;
  IRbeanSerial::start();
}

ISR (INT7_vect)  //Interrupcion para el IR2
{
  EIMSK &= ~(1 << INT2) & ~(1 << INT6) & ~(1 << INT7); //Las quito
  capturedBits=0;
  //ArduBot::ir2data=0;
  IRbeanSerial::start();
}

ISR (INT3_vect)  //Interrupcion para el Lect_Pulsador
{
  EIMSK &= ~(1 << INT3);
  delay(10);
  //Se comprueba el nivel del pin
  byte port=PIND & (1<<PD3);
  //if(port==0)
  //  unset_PWR_ON_OFF();
  EIMSK |= (1 << INT3);
}
//------------------------------------------------//
//--------Interrupcion para los motores-----------//
byte oldPort=0;
ISR ( PCINT1_vect )
{
  byte port=PINJ & ((1<<MOTOR_1_HALL_A_PIN) | (1<<MOTOR_1_HALL_B_PIN) | (1<<MOTOR_2_HALL_A_PIN) | (1<<MOTOR_2_HALL_B_PIN));
  if(port==oldPort)
    return;
  oldPort=port;
  boolean newLeftMotorHallA=((port&(1<<MOTOR_2_HALL_A_PIN))!=0);
  boolean newLeftMotorHallB=((port&(1<<MOTOR_2_HALL_B_PIN))!=0);
  boolean newRightMotorHallA=((port&(1<<MOTOR_1_HALL_A_PIN))!=0);
  boolean newRightMotorHallB=((port&(1<<MOTOR_1_HALL_B_PIN))!=0);
  
  if(newLeftMotorHallA!=ArduBot::leftMotorHallA)
  {
    if(newLeftMotorHallA==newLeftMotorHallB)
      ArduBot::leftMotor.pulsesDifference--;
    else
      ArduBot::leftMotor.pulsesDifference++;
    ArduBot::leftMotorHallA=newLeftMotorHallA;
  }
  else if(newLeftMotorHallB!=ArduBot::leftMotorHallB)
  {
    if(newLeftMotorHallB!=newLeftMotorHallA)
      ArduBot::leftMotor.pulsesDifference--;
    else
      ArduBot::leftMotor.pulsesDifference++;
    ArduBot::leftMotorHallB=newLeftMotorHallB;
  }
  
  if(newRightMotorHallA!=ArduBot::rightMotorHallA)
  {
    if(newRightMotorHallA==newRightMotorHallB)
      ArduBot::rightMotor.pulsesDifference++;
    else
      ArduBot::rightMotor.pulsesDifference--;
    ArduBot::rightMotorHallA=newRightMotorHallA;
  }
  else if(newRightMotorHallB!=ArduBot::rightMotorHallB)
  {
    if(newRightMotorHallB!=newRightMotorHallA)
      ArduBot::rightMotor.pulsesDifference++;
    else
      ArduBot::rightMotor.pulsesDifference--;
    ArduBot::rightMotorHallB=newRightMotorHallB;
  }
}
//----------------------------------------------------------//
//----------Inicializacion del sistema----------------------//
ArduBot::ArduBot(double wheelRadius, double wheelDistance, int encoderResolution) : lcd(4,20,0x63,0), isSrfUpdateContinuous(true),
    NUM_SRFs(0), NUM_SRFs_FRONT(0), NUM_SRFs_BACK(0), lcdState(false), energyState(false), accelGyro(0x68), accelGyroState(false)
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
  //xCoordinate=0;
  //yCoordinate=0;
  //thetaCoordinate=0;
  alert_stop=0;
  xCoordinateNoGyro=0;
  yCoordinateNoGyro=0;
  thetaCoordinateNoGyro=0;
//---------------------------------------------//
}
void ArduBot::begin(double spinLoopPeriodS, double kp, double ki, double kd)
{
  pinMode(CHARGER_ARDUINO_PIN,INPUT);
  pinMode(BATTERY_LEVEL_DIGITAL_INPUT_ARDUINO_PIN,INPUT);
//-----Inicializacion del puerto serie---------//
  Serial.begin(115200);
  Serial.flush();
//---------------------------------------------//

//-------Inicializacion del puerto I2C---------//
  Wire.begin();
  Wire.setClock(400000);
//---------------------------------------------//
  lcd.init();                          // Init the display, clears the display
  lcd.clear();
  lcd.home();
  delay(100);
  lcd.print("Calibrating...");
//---------------------------------------------//
  testSrfs();
  
  //Meto a machete los dos srf10 frontales para probar
  /*
  SRFs_FRONT[0]=112;
  SRFs_FRONT[1]=113;
  SRFs_FRONT[2]=114;
  SRFs_FRONT[3]=115;
  NUM_SRFs_FRONT=4;
  */
  
  //-------Inicializacion del MPU6050----------------//
  accelGyro.initialize();
  accelGyroState = accelGyro.testConnection();
  calibrate();
  delay(100);
  lcd.clear();
  lcd.home();
  lcd.print("Calibrated!");
  
  //-------Reading energy status---------------------//
  byte stat=0;
  byte value=0;
  energyState = true;
  getBatteryLevel(&value,&stat);
  lcd.setCursor(2,0);
  lcd.print("                    ");
  lcd.setCursor(2,0);
  lcd.print("Bat: ");
  float batV=((float)value/10);
  lcd.print(batV,1);
  lcd.print("V");
  lcd.setCursor(2,12);
  lcd.print("Etat: ");
  lcd.print(stat,DEC);


  //-------Init audio ampli---------------------//
  stat>>=1;
  stat&=0x01;
  if(stat==1)
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
    
  oldPort=PINJ & ((1<<MOTOR_1_HALL_A_PIN) | (1<<MOTOR_1_HALL_B_PIN) | (1<<MOTOR_2_HALL_A_PIN) | (1<<MOTOR_2_HALL_B_PIN));
  leftMotorHallA=((oldPort&(1<<MOTOR_1_HALL_A_PIN))!=0);
  leftMotorHallB=((oldPort&(1<<MOTOR_1_HALL_B_PIN))!=0);
  rightMotorHallA=((oldPort&(1<<MOTOR_2_HALL_A_PIN))!=0);
  rightMotorHallB=((oldPort&(1<<MOTOR_2_HALL_B_PIN))!=0);
  ArduBot::spinLoopPeriodMs=long(spinLoopPeriodS*1000);
  ArduBot::par_motores.initializePIDVariables(kp, kd, ki, spinLoopPeriodS);
  spinTime=srf10time=millis();
  IRbeanSerial::set(readIRByte);
  TCCR4B=TCCR4B & 0b11111000 | 0x01;  //Frecuencia del pwm al máximo para evitar el ruido en los motores (otra )
  PCMSK1 = 0 | (1 << PCINT11) | (1 << PCINT12) | (1 << PCINT13) | (1 << PCINT14);  //Pines que interrumpen para los encoders de los motores
  EIMSK |= (1 << INT2) | (1 << INT6) | (1 << INT7) | (1 << INT3); //Las pongo en marcha

}
//----------------------------------------------------------//
//-----------------Funciones para los encoders--------------//


long old_time=millis();
long stopTime=1000;
long speedComandTime=0;
void ArduBot::updatePosition()
{
  
  long now=millis();
  if(now>speedComandTime+stopTime)
  {
    setSpeeds(0,0);
  }
  
#ifdef WHEEL_STOP_ALERT
  if (ArduBot::wheelStopAlertFlag)
  {
    double linearSpeed=(ArduBot::par_motores.desiredLinearSpeed<0 ? ArduBot::par_motores.desiredLinearSpeed : 0);
    setSpeeds(linearSpeed,ArduBot::par_motores.desiredAngularSpeed);
  }
#endif
#ifdef ROBOT_FALL_ALERT
  if (ArduBot::robotFallFlag)
  {
    setSpeeds(0,0);
  }
#endif
#ifdef ROBOT_CRASH_ALERT
  if (ArduBot::robotCrashFlag)
  {
    double linearSpeed=(ArduBot::par_motores.desiredLinearSpeed<0 ? ArduBot::par_motores.desiredLinearSpeed : 0);
    setSpeeds(linearSpeed,ArduBot::par_motores.desiredAngularSpeed);
  }
#endif
#ifdef ROBOT_STALL_ALERT
  if (ArduBot::robotStallFlag)
  {
    setSpeeds(0,0);
  }
#endif
/*
  lcd.clear();
  lcd.print(ArduBot::par_motores.leftError);
  lcd.command(13);
  lcd.print(ArduBot::par_motores.rightError);
  
  lcd.command(13);
  lcd.print(ArduBot::par_motores.leftMotorPwm);
  lcd.command(13);
  lcd.print(ArduBot::par_motores.rightMotorPwm);
*/
  ArduBot::robotStallFlag=ArduBot::par_motores.doControlLoop();
  //lcd.clear();
  //lcd.println(stall,DEC);
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

void ArduBot::setSpeeds(double linear_speed, double angular_speed)
{
  ArduBot::par_motores.desiredLinearSpeed=linear_speed;
  ArduBot::par_motores.desiredAngularSpeed=angular_speed;
  ArduBot::par_motores.transformSpeeds2Pulses();
  speedComandTime=millis();
}
//----------------------------------------------------------//
//---------------Funcion para leer los SRF10----------------//
int ArduBot::updateSrfRange(int address){
  Wire.beginTransmission(address);                 // Start communticating with SRF10
  Wire.write(srfCmdByte);                             // Send Command Byte
  Wire.write(0x51);                                // Send 0x51 to start a ranging
  Wire.endTransmission();
}
unsigned int ArduBot::getSrfRange(int address){    // This function gets a ranging from the SRF10

  Wire.beginTransmission(address);                 // start communicating with SRFmodule
  Wire.write(srfRangeByte);                        // Call the register for start of ranging data
  Wire.endTransmission();
  
  Wire.requestFrom(address, 2);                    // Request 2 bytes from SRF module
  //while(I2c.available() < 2);                    // Wait for data to arrive

  int highByte = Wire.read();                      // Get high byte
  int lowByte = Wire.read();                       // Get low byte

  unsigned int range = (highByte << 8) + lowByte;  // Put them together
  //Serial.println(range,DEC);
  
  return(range);                                   // Returns Range
}

int ArduBot::getSrfLight(int address){             // Function to get light reading. Always 0x80 in the SRF10. Used to test the existance of the sensor
  
  Wire.beginTransmission(address);
  Wire.write(srfLightByte);                        // Call register to get light reading
  Wire.endTransmission();
  
  Wire.requestFrom(address, 1);                    // Request 1 byte
  //while(I2c.available() < 0);                    // While byte available
  int lightRead = Wire.read();                     // Get light reading
    
  return(lightRead);                               // Returns lightRead
  
}

int ArduBot::changeSrfAddress(int oldAddress, int newAddress)
{

  Wire.beginTransmission(oldAddress);
  Wire.write(srfCmdByte);                           // Send Command Byte
  Wire.write(0xA0);
  Wire.endTransmission();


  Wire.beginTransmission(oldAddress);
  Wire.write(srfCmdByte);
  Wire.write(0xAA);
  Wire.endTransmission();


  Wire.beginTransmission(oldAddress);
  Wire.write(srfCmdByte);
  Wire.write(0xA5);
  Wire.endTransmission();


  Wire.beginTransmission(oldAddress);
  Wire.write(srfCmdByte);
  Wire.write(newAddress);
  Wire.endTransmission();

}

int ArduBot::changeSrfGain(int address, int gain)
{

  Wire.beginTransmission(address);
  Wire.write(srfGainByte);                             // Send Command Byte
  Wire.write(gain);
  Wire.endTransmission();

}

int ArduBot::changeSrfDistance(int address, int range)
{

  Wire.beginTransmission(address);
  Wire.write(srfRangeByte);                             // Send Command Byte
  Wire.write(range);
  Wire.endTransmission();

}

void ArduBot::testSrfs()
{
  int address=0x70;  //Ojo con las direcciones. El LSB se quita y las direcciones aumentan de 1 en 1
  for(byte i=0;i<16;i++)
  {
    if(getSrfLight(address)==0x80)
    {
      SRFs_ADRESS[NUM_SRFs]=address;
      NUM_SRFs++;
    }
    address++;
  }
  setSrfsRegisters();
}

void ArduBot::setSrfsRegisters()
{
  for(int i=0;i<NUM_SRFs;i++)
  {

    // PARAMS FOR SRF10  DISTANCE & GAIN
    // distance (range) 0=43mm, 1=0.86m, 24=1m, 94=4m
    changeSrfDistance(SRFs_ADRESS[i],94);
    // Gain : 0 et 1=max analog gain de 40, 2=max analog gain de 50, 3=max analog gain de 60...
    changeSrfGain(SRFs_ADRESS[i],6);  //3
  }
}

//----------------------------------------------------------//
//boolean front=true;
byte index_front=0;
byte index_back=0;
//long testLCDtime=0;
//long testAMPLItime=0;
//long gyroTime=0;
long accelerometerTime=0;
long batteryTime=0;
//int ampliV=0;
//boolean filaLCD=false;
//char filaTestLCD[][80]={"  Thecorpora Robot  \n  =====Testing====","Esto es una prueba  del funcionamiento   del LCD\n- - - - - - - - - - "};
//long pulsos_izq=0;
//long pulsos_der=0;
byte accelerometer_fall_count = 0;

void ArduBot::spinOnce(){
  //Calculo de la posicion de l robot y procesamiento del PID
  long now=millis();
  if (now - spinTime > ArduBot::spinLoopPeriodMs)
  {
    spinTime += ArduBot::spinLoopPeriodMs;
    accelGyro.getMotion6(&accelerometerX, &accelerometerY, &accelerometerZ, &gyroX, &gyroY, &gyroZ);
    accelGyro.setDLPFMode(2); // +- 1000°/s
    roll = 180.0*accelerometerY/(accelerometerZ*3.1416);
    pitch = 180.0*accelerometerX/(accelerometerZ*3.1416);
    ArduBot::updatePosition();// give orders to motors
  }

  if(now-batteryTime>10000)
  {
    batteryTime+=10000;
    byte stat=0;
    byte value=0;
    /*
    Wire.requestFrom(0x14,2);
    byte stat=Wire.read();
    byte value=Wire.read();
    */
    energyState = true;
    getBatteryLevel(&value, &stat);
    lcd.setCursor(2,0);
    lcd.print("                    ");
    lcd.setCursor(2,0);
    lcd.print("Bat: ");
    float batV=((float)value/10);
    lcd.print(batV,1);
    lcd.print("V");
    lcd.setCursor(2,12);
    lcd.print("Etat: ");
    lcd.print(stat,DEC);
    lcd.setCursor(3,12);

    stat>>=1;
    stat&=0x01;
    if(stat==1)
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
  
  //Test del LCD
  /*if(now-testLCDtime>1000)
  {
    testLCDtime+=1000;
    lcd.clear();
    lcd.print(filaTestLCD[filaLCD]);
    filaLCD=!filaLCD;
  }*/
  //Test del Gyro
  /*
  if(now-gyroTime>200)
  {
    gyroTime+=200;
    //Get IMU data
    //gyro.getGyroValues(gyroX,gyroY,gyroZ);
    //accelerometer.getAccelerometerValues(accelerometerX,accelerometerY,accelerometerZ);
    //gyroZ-=gyro0;
    lcd.clear();
    lcd.print(xCoordinateNoGyro,DEC);
    lcd.command(13);
    lcd.print(yCoordinateNoGyro,DEC);
    lcd.command(13);
    lcd.print(thetaCoordinateNoGyro,DEC);
    lcd.command(13);
  }
  */
  //Test del AMPLI
  /*
  if(now-testAMPLItime>500)
  {
    testAMPLItime+=500;
    Wire.beginTransmission(B1001011);
    Wire.write(ampliV);                             // Send Command Byte
    Wire.endTransmission();
    ampliV=++ampliV%64;
  }
  */
  //Captura de los sensores de ultrasonido
  
  if(now-srf10time>130) //65
  {
    srf10time+=130;
    if (abs(ArduBot::par_motores.actualLeftWheelPulses)>0||abs(ArduBot::par_motores.actualRightWheelPulses)>0||isSrfUpdateContinuous)
    {
      if(NUM_SRFs_FRONT>0)
      {
      unsigned int srf10Range=getSrfRange(SRFs_FRONT[index_front]);
      SRFs_VALUES[SRFs_FRONT[index_front]-112]=srf10Range;
      /*
      Serial.print(SRFs_FRONT[index_front],DEC);
      Serial.print("\t");
      Serial.println(srf10Range,DEC);
      */
#ifdef ROBOT_CRASH_ALERT
      if((srf10Range!=0)&&(srf10Range<25))
      {
        SRFs_FRONT_CRASH_FLAGS[index_front]=true;
      }
      else if(SRFs_FRONT_CRASH_FLAGS[index_front])
      {
        SRFs_FRONT_CRASH_FLAGS[index_front]=false;
      }
      boolean crash=false;
      for(int z=0;z<NUM_SRFs_FRONT;z++)
      {
        if (SRFs_FRONT_CRASH_FLAGS[z])
        {
          crash=true;
          break;
        }
      }
      if(crash)
      {
        if(!ArduBot::robotCrashFlag)
        {
          lcd.setCursor(3,0);
          lcd.print("                    ");
          lcd.setCursor(3,0);
          lcd.print("Crash");
        }
        ArduBot::robotCrashFlag=true;
      }
      else if(ArduBot::robotCrashFlag==true)
      {
        lcd.setCursor(3,0);
        lcd.print("                    ");
        lcd.setCursor(3,0);
        lcd.print("Ready");
        ArduBot::robotCrashFlag=false;
      }
#endif
        index_front=(index_front+1)%NUM_SRFs_FRONT;
        updateSrfRange(SRFs_FRONT[index_front]);
      }
      if(NUM_SRFs_BACK>0)
      {
        SRFs_VALUES[SRFs_BACK[index_back]-112]=getSrfRange(SRFs_BACK[index_back]);
        index_back=(index_back+1)%NUM_SRFs_BACK;
        updateSrfRange(SRFs_BACK[index_back]);
      }
    }
  }
  
}
