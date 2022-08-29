/*********************************************************************
   \author Daniel Julian
 *********************************************************************/

#ifndef motores_h
#define motores_h

#include <inttypes.h>
#include <avr/io.h>
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

namespace arduBot
{
class CMotors {
  public:
    int pulsesDifference;
    int pwmValue;

    volatile uint8_t *control1Port;
    volatile uint8_t *control2Port;
    byte control1Pin;
    byte control2Pin;
    byte pwmPin;  //arduino form
    void setDirection()
    {
      if (pwmValue < 0)
      {
        bitClear(*control1Port, control1Pin);
        bitSet(*control2Port, control2Pin);
      }
      else
      {
        bitClear(*control2Port, control2Pin);
        bitSet(*control1Port, control1Pin);
      }
    };
    void pushPwm(int newPwmValue)
    {
      pwmValue = newPwmValue;
      setDirection();
      analogWrite(pwmPin, abs(pwmValue));
    };
    int getPulsesDifference()
    {
      int pulsesDifference = 0;
      ATOMIC_BLOCK(ATOMIC_FORCEON)
      {
        pulsesDifference = this->pulsesDifference;
        this->pulsesDifference = 0;
      }
      return pulsesDifference;
    };
    CMotors(volatile uint8_t *control1Register, volatile uint8_t *control2Register, volatile uint8_t *control1Port, volatile uint8_t *control2Port, byte control1Pin, byte control2Pin, byte pwmPin) :
      control1Port(control1Port), control2Port(control2Port), control1Pin(control1Pin), control2Pin(control2Pin), pwmPin(pwmPin)
    {
      pinMode(pwmPin, OUTPUT);
      bitSet(*control1Register, control1Pin);
      bitSet(*control2Register, control2Pin);
    };
};

class CBaseMovement {
  public:
    CMotors *leftMotor;
    CMotors *rightMotor;
    double desiredLinearSpeed;
    double desiredAngularSpeed;
    double actualLinearMovement;
    double actualAngularMovement;

    double desiredLinearPulses;
    double desiredAngularPulses;
    double referenceLinearPulses;
    double referenceAngularPulses;
    double referenceLeftWheelPulses;
    double referenceRightWheelPulses;
    double actualLeftWheelPulses;
    double actualRightWheelPulses;

    double linearError;
    double angularError;
    double leftError;
    double rightError;
    double integralLeftError;
    double integralRightError;
    int leftMotorPwm;
    int rightMotorPwm;

    double sampleTime;

    double kp;
    double ki;
    double kd;

    double maxLinearAcceleration;
    double maxAngularAcceleration;
    int encoderResolution;
    double wheelRadius;
    double distanceBetweenWheels;
    double linearPulse2LinearMovementConstant;
    double angularPulse2AngularMovementConstant;

    byte variablesInitialized;

    CBaseMovement(CMotors *leftMotor, CMotors *rightMotor, double kp = 0, double kd = 0, double ki = 0, double sampleTime = 0.005, int encoderResolution = 1440, double distanceBetweenWheels = 0.2736, double wheelRadius = 0.102) : leftMotor(leftMotor), rightMotor(rightMotor),
      kp(kp), kd(kd), ki(ki), sampleTime(sampleTime), encoderResolution(encoderResolution), distanceBetweenWheels(distanceBetweenWheels), wheelRadius(wheelRadius)
    {
      desiredLinearSpeed = 0;
      desiredAngularSpeed = 0;
      actualLinearMovement = 0;
      actualAngularMovement = 0;

      integralLeftError = 0;
      leftError = 0;
      integralRightError = 0;
      rightError = 0;

      maxLinearAcceleration = 1; //1m/ss
      maxAngularAcceleration = 3;

      if (encoderResolution == 0)
      {
        variablesInitialized = 0;
        return;
      }
      variablesInitialized = 1;
      linearPulse2LinearMovementConstant = PI * wheelRadius / encoderResolution;
      angularPulse2AngularMovementConstant = 2 * PI * wheelRadius / (distanceBetweenWheels * encoderResolution);
    };

    void initializePhisicalVariables(int _encoderResolution, double _distanceBetweenWheels, double _wheelRadius)
    {
      encoderResolution = _encoderResolution;
      distanceBetweenWheels = _distanceBetweenWheels;
      wheelRadius = _wheelRadius;
      if (encoderResolution == 0)
      {
        variablesInitialized = 0;
        return;
      }
      variablesInitialized = 1;
    };

    void initializePIDVariables(double _kp, double _kd, double _ki, double _sampleTime)
    {
      kp = _kp;
      kd = _kd;
      ki = _ki;
      sampleTime = _sampleTime;
    };

    void transformSpeeds2Pulses()
    {
      if (variablesInitialized == 0)
        return;
      desiredLinearPulses = desiredLinearSpeed * sampleTime / linearPulse2LinearMovementConstant;
      desiredAngularPulses = desiredAngularSpeed * sampleTime / angularPulse2AngularMovementConstant;
    };

    void limitAcceleration()
    {
      double accelerationGap = desiredLinearPulses - referenceLinearPulses;
      if (abs(accelerationGap) < maxLinearAcceleration)
        referenceLinearPulses = desiredLinearPulses;
      else
      {
        if (accelerationGap > 0)
          referenceLinearPulses += maxLinearAcceleration;
        else
          referenceLinearPulses -= maxLinearAcceleration;
      }

      accelerationGap = desiredAngularPulses - referenceAngularPulses;
      if (abs(accelerationGap) < maxAngularAcceleration)
        referenceAngularPulses = desiredAngularPulses;
      else
      {
        if (accelerationGap > 0)
          referenceAngularPulses += maxAngularAcceleration;
        else
          referenceAngularPulses -= maxAngularAcceleration;
      }
    };

    void linearPlusAngular2LeftPlusRight()
    {
      referenceRightWheelPulses = (referenceLinearPulses + referenceAngularPulses) / 2;
      referenceLeftWheelPulses = (referenceLinearPulses - referenceAngularPulses) / 2;
    };

    void doPIDControl()
    {
      integralLeftError += leftError;
      integralRightError += rightError;
      double oldLeftError = leftError;
      double oldRightError = rightError;
      leftError = actualLeftWheelPulses - referenceLeftWheelPulses;
      rightError = actualRightWheelPulses - referenceRightWheelPulses;
      double derivativeLeftError = leftError - oldLeftError;
      double derivativeRightError = rightError - oldRightError;
      leftMotorPwm = kp * leftError + kd * derivativeLeftError + ki * integralLeftError;
      rightMotorPwm = kp * rightError + kd * derivativeRightError + ki * integralRightError;
    };

    int checkMotorsLimits()
    {
      int limitStatus = 0;
      if ((abs(leftMotorPwm) > 255) || (abs(rightMotorPwm) > 255))
      {
        int maxMotorPwm = max(abs(leftMotorPwm), abs(rightMotorPwm));
        leftMotorPwm = ((double)leftMotorPwm * 255) / maxMotorPwm;
        rightMotorPwm = ((double)rightMotorPwm * 255) / maxMotorPwm;
        limitStatus = 1;
      }
      return limitStatus;
    };

    void updateMotorsPwm()
    {
      leftMotor->pushPwm(leftMotorPwm);
      rightMotor->pushPwm(rightMotorPwm);
    };

    void updateMotorsMovement()
    {
      actualLeftWheelPulses = leftMotor->getPulsesDifference();
      actualRightWheelPulses = rightMotor->getPulsesDifference();
      actualLinearMovement = (actualLeftWheelPulses + actualRightWheelPulses) * linearPulse2LinearMovementConstant;
      actualAngularMovement = (actualRightWheelPulses - actualLeftWheelPulses) * angularPulse2AngularMovementConstant;
    };

    bool doControlLoop()
    {
      if (variablesInitialized == 0)
        return 0;

      updateMotorsMovement();
      if ((desiredLinearPulses == 0) && (desiredAngularPulses == 0))
      {
        freeMotors();
        return 0;
      }

      limitAcceleration();
      linearPlusAngular2LeftPlusRight();
      doPIDControl();
      checkMotorsLimits();
      updateMotorsPwm();
      return 0;
    };

    void freeMotors()
    {
      desiredLinearSpeed = 0;
      desiredAngularSpeed = 0;
      linearPlusAngular2LeftPlusRight();

      integralLeftError = 0;
      leftError = 0;
      integralRightError = 0;
      rightError = 0;
      leftMotor->pushPwm(0);
      rightMotor->pushPwm(0);
    };
};
}

#endif
