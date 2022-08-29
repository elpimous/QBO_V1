/*********************************************************************
   \author Miguel Angel Julian  MODDED 10 juin 2019
 *********************************************************************/

#include "ArduBot.h"
using namespace arduBot;

//---------Definition des variables static------------//
// states of encoders
bool ArduBot::leftMotorHallA = LOW;
bool ArduBot::leftMotorHallB = LOW;
bool ArduBot::rightMotorHallA = LOW;
bool ArduBot::rightMotorHallB = LOW;

// states of IR sensors
byte ArduBot::ir1data = 0;
byte ArduBot::ir2data = 0;
byte ArduBot::ir3data = 0;

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

//--------Interruptions pour les moteurs-----------//
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
//------------------------------------------------//

//----------Initialisation systeme---------------//
ArduBot::ArduBot(double wheelRadius, double wheelDistance, int encoderResolution) : lcdState(false), energyState(false), accelGyro(0x68), accelGyroState(false)
{
  //----------Configuration des pins de la board---------//
  DDRJ &= ~(1 << MOTOR_1_HALL_A_PIN) & ~(1 << MOTOR_1_HALL_B_PIN) & ~(1 << MOTOR_2_HALL_A_PIN) & ~(1 << MOTOR_2_HALL_B_PIN);  //Pines de los sensores HALL como entradas
  PORTJ |= (1 << MOTOR_1_HALL_A_PIN) | (1 << MOTOR_1_HALL_B_PIN) | (1 << MOTOR_2_HALL_A_PIN) | (1 << MOTOR_2_HALL_B_PIN);  //Resistencias de pull-up para sensores hall
  //---------------------------------------------//
  //-----Interruptions pour les moteurs---------//
  PCICR |= (1 << PCIE1);  //Habilito las interrupciones de los motores
  //---------------------------------------------//
  
  //-----Interruptions pour les IRS--------------//
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
  
  //-----Initialisation du port serie---------//
  Serial.begin(115200);
  Serial.flush();
  
  //-------Initialisation du port I2C---------//
  Wire.begin();
  Wire.setClock(400000);

  //-------Inicializacion del LCD----------------//
  lcd.begin(20,4);
  lcd.clear();
  delay(100);
  lcd.backlight();
  lcd.setCursor(4,1);
  lcd.print("Welcome...");
  delay(1200); // 1.2s
  
  //-------Initialisation du MPU6050 (IMU)---------//
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(2,0);
  lcd.print("IMU CALIBRATION.");
  lcd.setCursor(0,1);
  lcd.print("--------------------");
  lcd.setCursor(1,2);
  lcd.print("Do not move robot.");
  
  accelGyro.initialize();
  accelGyroState = accelGyro.testConnection();
  calibrate();
  delay(100);
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(5,1);
  lcd.print("Ready...");
  delay(1200);

  //-------Lecture des valeurs batterie----------//
  byte stat = 0;
  byte value = 0;
  energyState = true;
  getBatteryLevel(&value, &stat);
  
  //-------Initialisation de l'ampli audio----------//
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
  TCCR4B = TCCR4B & 0b11111000 | 0x01; //Fréquence maximale du pwm pour éviter le bruit dans les moteurs (autre)
  PCMSK1 = 0 | (1 << PCINT11) | (1 << PCINT12) | (1 << PCINT13) | (1 << PCINT14);  //Pins interrompant les encodeurs de moteurs
  EIMSK |= (1 << INT2) | (1 << INT6) | (1 << INT7) | (1 << INT3); //Mise en marche !!!
}
//----------------------------------------------------------//

//----------------Fonctions pour les encodeurs--------------//


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
  float odometryLinearMovement = ArduBot::par_motores.actualLinearMovement;
  float odometryAngularMovement = 1.0256*ArduBot::par_motores.actualAngularMovement; // coefficient de correction (360/351)
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
  lcd.clear();
  lcd.noBacklight(); // power on backlight
  
  // Avertissement du FLOOR SENSOR : mur
  if(MiddleDist<MIN_FLOOR_DISTANCE_CM){
    
    byte wall_1[8] = {
  0b00001,0b00010,0b00100,0b01001,0b11110,0b10010,0b10010,0b10010
  };
    byte wall_2[8] = {
  0b11000,0b01000,0b10000,0b01000,0b01010,0b01110,0b01111,0b11111
  };
    byte wall_3[8] = {
  0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b11111,0b11111
  };
    byte wall_4[8] = {
  0b10010,0b10010,0b10010,0b10010,0b10010,0b10010,0b10010,0b11110
  };
    byte wall_5[8] = {
  0b11111,0b01111,0b01110,0b01010,0b01000,0b10000,0b00000,0b00000
  };
    byte wall_6[8] = {
  0b11111,0b11111,0b00000,0b00000,0b00000,0b00000,0b00000,0b00000
  };
    lcd.backlight();
    lcd.setCursor(6,0); // colonnes, lignes
    lcd.print("|");
    lcd.setCursor(6,1);
    lcd.print("|");
    lcd.setCursor(6,2);
    lcd.print("|");
    lcd.setCursor(6,3);
    lcd.print("|"); 
    lcd.setCursor(10,0);
    lcd.print("ALERT :");
    lcd.setCursor(7,1);
    lcd.print("-------------");
    lcd.setCursor(11,2);
    lcd.print("WALL");
    lcd.setCursor(1,1);
    lcd.createChar(0, wall_1);
    lcd.write(0);
    lcd.setCursor(2,1);
    lcd.createChar(1, wall_2);
    lcd.write(1);
    lcd.setCursor(3,1);
    lcd.createChar(2, wall_3);
    lcd.write(2);
    lcd.setCursor(1,2);
    lcd.createChar(3, wall_4);
    lcd.write(3);
    lcd.setCursor(2,2);
    lcd.createChar(4, wall_5);
    lcd.write(4);
    lcd.setCursor(3,2);
    lcd.createChar(5, wall_6);
    lcd.write(5);
    if(linear_speed>0){
      linear_speed = 0.0;
    }
  }

  // Avertissement du FLOOR SENSOR : chute
  if(MiddleDist>MAX_FLOOR_DISTANCE_CM){
    
    byte fall_1[8] = {
  0b00000,0b00000,0b00001,0b00011,0b10111,0b11110,0b11100,0b11110
  };
    byte fall_2[8] = {
  0b00000,0b00001,0b11110,0b11111,0b11111,0b01001,0b10001,0b00000
  };
    byte fall_3[8] = {
  0b00000,0b10000,0b01000,0b00100,0b10010,0b11011,0b10000,0b00001
  };
    byte fall_4[8] = {
  0b10000,0b11000,0b10100,0b10010,0b10001,0b01001,0b00101,0b00011
  };
    byte fall_5[8] = {
  0b00101,0b01001,0b10001,0b00001,0b00010,0b00100,0b01000,0b10000
  };
    lcd.backlight();
    lcd.setCursor(6,0);
    lcd.print("|");
    lcd.setCursor(6,1);
    lcd.print("|");
    lcd.setCursor(6,2);
    lcd.print("|");
    lcd.setCursor(6,3);
    lcd.print("|"); 
    lcd.setCursor(10,0);
    lcd.print("ALERT :");
    lcd.setCursor(7,1);
    lcd.print("-------------");
    lcd.setCursor(11,2);
    lcd.print("FALL");
    lcd.setCursor(1,1);
    lcd.createChar(0, fall_1);
    lcd.write(0);
    lcd.setCursor(2,1);
    lcd.createChar(1, fall_2);
    lcd.write(1);
    lcd.setCursor(3,1);
    lcd.createChar(2, fall_3);
    lcd.write(2);
    lcd.setCursor(2,2);
    lcd.createChar(3, fall_4);
    lcd.write(3);
    lcd.setCursor(3,2);
    lcd.createChar(4, fall_5);
    lcd.write(4);
    if(linear_speed>0){
      linear_speed = 0.0;
    }
  }
  /*
  // Avertissement de l'IMU robot par terre
  if(accelerometerZ<16900){
    lcd.setCursor(1, 0);
    lcd.print("   Alerte : chute   ");
    //linear_speed = 0.0;
    //angular_speed = 0.0;
  }
  */
  
  /*
  // Avertissement du FRONT LEFT SRF
  if(LeftDist<20){
    lcd.setCursor(1, 0);
    lcd.print("Alert : Crash");
    if(linear_speed>0){
      linear_speed = 0.0;
    }
  }

  // Avertissement du FRONT RIGHT srf
  if(RightDist<20){
    lcd.setCursor(1, 0);
    lcd.print("Alert : Crash");
    if(linear_speed>0){
      linear_speed = 0.0;
    }
  }
  */



  ArduBot::par_motores.desiredLinearSpeed = linear_speed;
  ArduBot::par_motores.desiredAngularSpeed = angular_speed;
  ArduBot::par_motores.transformSpeeds2Pulses();
  speedComandTime = millis();
}
unsigned int ArduBot::getSrfRange(int address) {

}

//----------------------------------------------------------//
//boolean front=true;
byte index_front = 0;
byte index_back = 0;
long accelerometerTime = 0;
long batteryTime = 0;
byte accelerometer_fall_count = 0;

void ArduBot::spinOnce() {

 
  //Calcul du positionnement du robot et du traitement du PID
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
    lcd.setCursor(2, 12); // ligne 2, colonne 12
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
