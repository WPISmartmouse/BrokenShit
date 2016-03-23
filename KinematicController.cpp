#include "KinematicController.h"
#include <math.h>

KinematicController::KinematicController(RegulatedMotor* leftMotor,
    RegulatedMotor* rightMotor,
		int leftMotorDirection, int rightMotorDirection,
		float wheelDistance, float wheelDiameter, unsigned int encoderCPR)
  : leftMotor(leftMotor),
  rightMotor(rightMotor),
  leftMotorDirection(leftMotorDirection),
  rightMotorDirection(rightMotorDirection),
  wheelDiameter(wheelDiameter),
  wheelDistance(wheelDistance),
  encoderCPR(encoderCPR),
  sampleTime(100) {
  setSampleTime(sampleTime);
}

void KinematicController::calibrate(uint16_t wheelDistance, uint16_t wheelDiameter){
	this->wheelDistance = wheelDistance;
	this->wheelDiameter = wheelDiameter;
}

void KinematicController::setSampleTime(unsigned long sampleTime){
  leftMotor->setSampleTime(sampleTime);
  rightMotor->setSampleTime(sampleTime);
}

void KinematicController::setAcceleration(unsigned int forwardAcceleration,
  unsigned int ccwAcceleration, unsigned int forwardDeceleration,
  unsigned int ccwDeceleration){

	this->forwardAcceleration = mmToTick(forwardAcceleration);
	this->ccwAcceleration = degToTick(ccwAcceleration);
	this->forwardDeceleration = mmToTick(forwardDeceleration);
	this->ccwDeceleration = degToTick(ccwDeceleration);

	atomicForwardAcceleration = (long)forwardAcceleration * sampleTime / 1000;
  atomicForwardDeceleration = (long)forwardDeceleration * sampleTime / 1000;
	atomicCCWAcceleration = (long)ccwAcceleration * sampleTime / 1000;
	atomicCCWDeceleration = (long)ccwDeceleration * sampleTime / 1000;
}


boolean KinematicController::run(){
	unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastRunTime;

  Serial.print(sampleTime);
  Serial.print(" ");
  Serial.print(deltaTime);

	if (deltaTime >= sampleTime){

    standby = true;

    if (currentTime - lastRamp < STANDBY_DELAY) standby = false;

    long forwardOutput = 0;
    long ccwOutput = 0;

    if (state == ControllerState::VELOCITY){
      if (lastForwardVelocity == targetForwardVelocity) {
        forwardOutput = targetForwardVelocity;
      }
      else {
        forwardOutput = speedRamp(lastForwardVelocity, targetForwardVelocity, atomicForwardAcceleration, atomicForwardDeceleration);
        standby = false;
        lastRamp = currentTime;
      }

      if (lastCCWVelocity == targetCCWVelocity) {
        ccwOutput = targetCCWVelocity;
      }
      else {
        ccwOutput = speedRamp(lastCCWVelocity, targetCCWVelocity, atomicCCWAcceleration, atomicCCWDeceleration);
        standby = false;
        lastRamp = currentTime;
      }

      leftMotor->setSpeed(calculateLeftWheelSpeed(forwardOutput, ccwOutput));
      rightMotor->setSpeed(calculateRightWheelSpeed(forwardOutput, ccwOutput));

      leftMotor->runNow(deltaTime);
      rightMotor->runNow(deltaTime);

      lastForwardVelocity = forwardOutput;
      lastCCWVelocity = ccwOutput;
    } else if (state == ControllerState::OFF) {
      lastForwardVelocity = 0;
      lastCCWVelocity = 0;
    }

    long localX = this->getOdometryForward();
    long localTheta = this->getOdometryCCW();
    globalX += (float)(localX-lastLocalX)*cos(localTheta*M_PI/180);
    globalY += (float)(localX-lastLocalX)*sin(localTheta*M_PI/180);
    lastRunTime = currentTime;
    lastLocalX = localX;
    lastLocalTheta = localTheta;

    return true;
  }

  return false;
}

void KinematicController::getGlobalPosition(long *x, long *y){
	*x = round(globalX);
	*y = round(globalY);
}

void KinematicController::goVelocity(int forwardVelocity, int ccwVelocity){
	state = ControllerState::VELOCITY;
	standby = false;
	targetForwardVelocity = mmToTick(forwardVelocity);
	targetCCWVelocity = degToTick(ccwVelocity);
}

void KinematicController::goPosition(int forwardDistance, int ccwAngle, unsigned int forwardSpeed, unsigned int ccwSpeed){
	state = ControllerState::POSITION;
	_goPosition(forwardDistance, ccwAngle, forwardSpeed, ccwSpeed);
}

void KinematicController::brake(){
	state = ControllerState::OFF;
	leftMotor->setState(RegulatedMotor::MotorState::BRAKE);
	rightMotor->setState(RegulatedMotor::MotorState::BRAKE);
}

void KinematicController::coast(){
	state = ControllerState::OFF;
	leftMotor->setState(RegulatedMotor::MotorState::COAST);
	rightMotor->setState(RegulatedMotor::MotorState::COAST);
}

void KinematicController::_goPosition(int forwardDistance, int ccwAngle, unsigned int forwardSpeed, unsigned int ccwSpeed){
	originTime = millis();
	originForwardTick = calculateForwardTick();
	originCCWTick = calculateCCWTick();
	targetForwardTick = mmToTick(forwardDistance);
	targetCCWTick = degToTick(ccwAngle);
	positionForwardVelocity = forwardSpeed;
	positionCCWVelocity = ccwSpeed;
}


long KinematicController::mmToTick(long mm){
	return (mm*encoderCPR)/(M_PI*wheelDiameter);
}

long KinematicController::degToTick(long deg){
	return mmToTick(deg*wheelDistance*M_PI/180)/2;
}

long KinematicController::calculateLeftWheelSpeed(long forwardVelocity, long ccwVelocity){
	long s = (forwardVelocity - ccwVelocity)*leftMotorDirection;
  return s;
}

long KinematicController::calculateRightWheelSpeed(long forwardVelocity, long ccwVelocity){
	long s = (forwardVelocity + ccwVelocity)*rightMotorDirection;
  return s;
}

long KinematicController::calculateForwardTick(){
	return (leftMotor->getEncoder()*leftMotorDirection + rightMotor->getEncoder()*rightMotorDirection)/2;
}

long KinematicController::calculateCCWTick(){
	return (-leftMotor->getEncoder()*leftMotorDirection + rightMotor->getEncoder()*rightMotorDirection);
}

long KinematicController::speedRamp(long last, long target,long up, long down){
	long ret;
    if (last >= 0){
        if (target > last){
            ret = last + up;
            if (ret > target) ret = target;
        } else {
            ret = last - down;
            if (ret < target) ret = target;
        }
    } else { // last < 0
        if (target > last){
            ret = last + down;
            if (ret > target) ret = target;
        } else {
            ret = last - up;
            if (ret < target) ret = target;
        }
    }
    return ret;
}

boolean KinematicController::isStandby(){
	return standby;
}

long KinematicController::getOdometryForward(){
	return calculateForwardTick()* (M_PI*wheelDiameter) / encoderCPR;
}

long KinematicController::getOdometryCCW(){
	return (calculateCCWTick() * (M_PI*wheelDiameter) / encoderCPR)/(wheelDistance*M_PI/180);
}
