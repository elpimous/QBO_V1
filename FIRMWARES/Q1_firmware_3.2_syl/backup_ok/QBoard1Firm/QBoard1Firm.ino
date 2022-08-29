/*********************************************************************
   \author Miguel Angel Julian REFERENCE
 *********************************************************************/

//---------------includes de las librerias usadas--------------//

#include "ArduBot.h"
#include "serialProtocol.h"

//-------------------------------------------------------------//

arduBot::ArduBot QBO(0.102, 0.2736, 1440); // wheel radius(m), distance between wheel(m), encoder resolution(step/rev)
arduBotSerial::SerialProtocol serialProtocol(&QBO);

//----------Inicializacion del sistema----------------------//
void setup()
{
  QBO.begin(0.005, 9.0, 1.0, 0.2); //Ts,kp,kd,ki

}
//----------------------------------------------------------//
//----------------------Loop principal----------------------//
void loop()
{
  serialProtocol.processSerial();
  QBO.spinOnce();
}
//----------------------------------------------------------//
