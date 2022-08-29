/*********************************************************************
 * \author Miguel Angel Julian
 *********************************************************************/

#ifndef SerialProtocol_h
#define SerialProtocol_h

#include "comands.h"
#include "ArduBot.h"

namespace arduBotSerial
{
  class CCommand
  {
    public:
      byte commandNumber;
      byte nInputData;
      byte inputData[128];
      byte nOutputData;
      byte outputData[128];
      CCommand()
      {
        commandNumber=0;
        nInputData=0;
        nOutputData=0;
      };
  };
  class SerialProtocol {
    private:
      const byte INPUT_FLAG;
      const byte OUTPUT_FLAG;
      const byte INPUT_ESCAPE;
      CCommand command_;
      bool isInputEscaped_;
      bool isInputCorrect_;
      arduBot::ArduBot* robot;

      void processCommands();
      boolean procesaEntrada(byte* buf, byte length);
      void sendResponse();
      byte sendDataByte(byte* data, byte nInputData, byte crc);
      byte sendDataByte(byte nInputData, byte crc);
      void sendNack();
    public:
      SerialProtocol(arduBot::ArduBot *robot);
      void processSerial();
  };
}

#endif
