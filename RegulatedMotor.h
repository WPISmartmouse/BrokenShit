#ifndef RegulatedMotor_h
#define RegulatedMotor_h
#define LIBRARY_VERSION	2.0.0
#include "../Encoder/Encoder.h"

#define MOTORSTATE_PWM	0
#define MOTORSTATE_SPEED	1
#define MOTORSTATE_BRAKE	3
#define MOTORSTATE_COAST	4

class RegulatedMotor
{
public:
	RegulatedMotor(long* encoder, int fwdPin, int revPin, int pwmPin);
	RegulatedMotor(long* encoder, int fwdPin, int revPin);
	bool run();
  void runNow(unsigned long deltaTime);
	void setSpeed(int speed);
	void setState(int state);
	void setPID(float kp, float ki, float kd, float kvff);
	void setSampleTime(unsigned long sampleTime);
	void goPWM(int pwm);
	long getEncoder();
	int getError();
private:

  const int MAX_ACCEL = 5;

	long* encoder;

	long thisPosition;
	long lastPosition;

	int calculatedSpeed;
	int lastCalculatedSpeed;
	int targetSpeed;

	int outputValue;

	float kp;
	float ki;
	float kd;
	float kvff;

	int iTerm;

	unsigned long lastTime;
  int lastOutput;

	unsigned long sampleTime; //MILLISECONDS

	int state;
	int lastState;

	int fwdPin;
	int revPin;
	int pwmPin;

	int error;
	//int getPWM(int speed);

};
#endif
