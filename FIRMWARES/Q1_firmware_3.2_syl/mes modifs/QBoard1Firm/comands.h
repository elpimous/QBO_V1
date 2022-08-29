/*********************************************************************
 * \author Miguel Angel Julian
 *********************************************************************/

#define GET_VERSION 0x40
#define ALL_SRF_UPDATE 0x41       //A
//#define GET_BRUJULA 0x42          //B  //Ya no está
#define RESET_ODOMETRY 0x42
#define CHANGE_I2C_DIR 0x43       //C
//#define SET_MOUTH_VALUE 0x44      //D  //Para la PR2
#define RESET 0x45                //E
#define GET_BASE_INFRARED 0x46    //IR
#define GET_I2C 0x47              //G
#define GET_GP2D12 0x48           //H
#define SRF_UPDATE 0x49           //I
//#define SET_MIC_INPUT 0x4A        //J  //Para la PR2
//#define GET_MIC_REPORT 0x4B       //K  //Para la PR2
#define SET_LCD 0x4C              //L
#define SET_MOTOR 0x4D            //M
#define GET_ALL_SRF 0x4E          //N
#define GET_ENCODERS 0x4F         //O
#define SET_PID 0x50              //PID
//#define NEW_FIRMWARE_PR1 0x51     //Q
#define GET_WEEL_MOVEMENT 0x51
#define GET_SRF 0x52              //R
//#define SET_SERVO 0x53            //S
#define TEST 0x54                 //TEST
#define SET_I2C 0x55              //U
#define GET_SPEED 0x56            //V
#define GET_BATTERY 0x57          //W
#define I2C_EXTENDED 0x58         //X
//#define NEW_FIRMWARE_PR2 0x59     //Y
#define GET_POSITION 0x59
//#define TURN_OFF 0x5A             //Z
//-----
//#define SET_MOTOR_EASY 0x5B
//-----
//#define GET_SERVOS 0x5C
#define SET_SRF_CONTINUOUS_UPDATE 0x5D
//#define SET_RADIOCONTROL 0x63     //CTRL
#define SET_ENCODER 0x70          //P
#define SET_K 0x71
#define SET_AUTOUPDATE_SRFS 0x72
#define GET_ANALOG_PINS 0x73
#define GET_IMU 0x74
#define RESET_STALL 0x80
#define GET_MOTORS_STATE 0x81
#define GET_I2C_STATE 0x82
