#include "Arduino.h"
#include <RegulatedMotor.h>
#include "../Encoder/Encoder.h"

RegulatedMotor::RegulatedMotor(long* _encoder, int _fwdPin, int _revPin, int _pwmPin) : thisPosition(0), lastPosition(0), calculatedSpeed(0), lastCalculatedSpeed(0), lastOutput(0)
{
	encoder = _encoder;
	fwdPin = _fwdPin;
	revPin = _revPin;
	pwmPin = _pwmPin;
	state = MOTORSTATE_COAST;
  lastTime = millis();
}

RegulatedMotor::RegulatedMotor(long* _encoder, int _fwdPin, int _revPin) : thisPosition(0), lastPosition(0), calculatedSpeed(0), lastCalculatedSpeed(0), lastOutput(0)

{
  encoder = _encoder;
  fwdPin = _fwdPin;
  revPin = _revPin;
  state = MOTORSTATE_COAST;
  lastTime = millis();
}


void RegulatedMotor::setPID(float _kp, float _ki, float _kd, float _kvff)
{
	kp = _kp;
	ki = _ki;
	kd = _kd;
	kvff = _kvff;
}

void RegulatedMotor::setSampleTime(unsigned long _sampleTime)
{
	sampleTime = _sampleTime;
}

void RegulatedMotor::setSpeed(int speed){
	state = MOTORSTATE_SPEED;
	targetSpeed = speed;
}

bool RegulatedMotor::run(){
  if (state == MOTORSTATE_PWM){
    return true;
  }

	if (state == MOTORSTATE_COAST){
		goPWM(0);
		lastState = MOTORSTATE_COAST;
		return true;
	}

	if (state == MOTORSTATE_BRAKE) {
    digitalWrite(fwdPin,255);
    digitalWrite(revPin,255);
    lastState = MOTORSTATE_BRAKE;
    return true;
	}

	unsigned long thisTime = millis();
	unsigned long deltaTime = thisTime - lastTime;


  if(deltaTime>=sampleTime){
    runNow(deltaTime);
    lastTime = thisTime;

	  return true;
  }
  return false;
}

void RegulatedMotor::runNow(unsigned long deltaTime){
    const int outMax = 255;
    const int outMin = -255;
    thisPosition = *encoder;

    if (lastState == MOTORSTATE_COAST || lastState == MOTORSTATE_BRAKE){
    	calculatedSpeed = 0;
    	iTerm = 0;
    } else {
    	calculatedSpeed = ((thisPosition - lastPosition) * 1000L) / ((long) deltaTime);
    }

    error = targetSpeed - calculatedSpeed;
    iTerm += (ki * error);

    if (iTerm > outMax) {
      iTerm = outMax;
    }
    else if (iTerm < outMin) {
      iTerm = outMin;
    }

    int dTerm = (kd *(calculatedSpeed - lastCalculatedSpeed)) / ((long) deltaTime);

    int output = constrain(kp * error + iTerm - dTerm + kvff * targetSpeed,outMin,outMax);

	  goPWM(output);

    lastCalculatedSpeed = calculatedSpeed;
    lastOutput = output;
    lastPosition = thisPosition;
    lastState = MOTORSTATE_SPEED;
}

void RegulatedMotor::goPWM(int pwm){
  int apwm = constrain(abs(pwm),0,255);
  if (pwm >= 0){
    analogWrite(revPin,0);
    analogWrite(fwdPin,apwm);
    return;
  }
  if (pwm < 0){
    analogWrite(fwdPin,0);
    analogWrite(revPin,apwm);
    return;
  }
}

void RegulatedMotor::setState(int _state){
	state = _state;
}

long RegulatedMotor::getEncoder(){
  long tmp = *encoder;
}

int RegulatedMotor::getError(){
  return error;
}
