/* -----------------------------------------------------------------------------
                                  MIT License

                        Copyright (c) 2018 Jason McKinney

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.

--------------------------------------------------------------------------------
	motionPlanner.c

	Created:  2017-11-09

	Minor Revisions:
	-	v1.0.0  Initial Release

--------------------------------------------------------------------------------
	The author asks that proper attribution be given for this software should the
	source be unavailable (for example, if compiled into a binary/used on a robot).

	The author can be contacted via email at jason_at_jmmckinney_dot_net
	or on the VEX Forum as jmmckinney.
-------------------------------------------------------------------------------- */


#ifndef TRUESPEED
#define TRUESPEED

/**
 * TrueSpeed lookup table maps linear motor input to logarithmic motor output in order
 * to improve motor control
 */
const unsigned int TrueSpeed[128] =
{
  0, 0, 10, 10, 10, 10, 11, 11, 11, 11,
  11, 12, 12, 12, 12, 12, 13, 13, 13, 13,
  13, 13, 14, 14, 14, 14, 14, 15, 15, 15,
  15, 16, 16, 16, 16, 16, 16, 17, 17, 17,
  17, 17, 18, 18, 18, 18, 19, 19, 19, 19,
  20, 20, 20, 20, 20, 21, 21, 21, 21, 21,
  22, 22, 22, 23, 23, 23, 24, 24, 24, 24,
  25, 25, 25, 26, 26, 26, 27, 27, 27, 28,
  28, 28, 29, 29, 30, 31, 31, 32, 32, 33,
  33, 34, 34, 34, 35, 35, 36, 37, 38, 40,
  41, 41, 42, 43, 44, 45, 47, 48, 50, 51,
  51, 52, 52, 53, 55, 56, 57, 58, 60, 62,
  66, 69, 74, 84, 85, 85, 126, 127
};
#endif


/*--- PID Implementation ---*/


#ifndef NERD_PID
#define NERD_PID

/**
 * PID controller data structure
 */
typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float innerIntegralBand;
	float outerIntegralBand;
	float sigma;
	float lastValue;
	unsigned long lastTime;
	float lastSetPoint;
} PID;

/**
 * initialize pid structure, set parameters
 *
 * pid instance of PID structure
 * Kp  proportional gain
 * Ki  integral gain
 * Kd  derivative gain
 * innerIntegralBand  inner bound of PID I summing cutoff
 * outerIntegralBand  outer bound of PID I summing cutoff
 */
void
pidInit (PID pid, float Kp, float Ki, float Kd, float innerIntegralBand, float outerIntegralBand) {
	pid.Kp = Kp;
	pid.Ki = Ki;
	pid.Kd = Kd;
	pid.innerIntegralBand = innerIntegralBand;
	pid.outerIntegralBand = outerIntegralBand;
	pid.sigma = 0;
	pid.lastValue = 0;
	pid.lastTime = nPgmTime;
}

/**
 * initialize pid structure, set parameters based on another PID structure
 *
 * @param pid  instance of PID structure
 * @param toCopy  PID instance to copy settings from
 */
void pidInitCopy (PID pid, PID toCopy) {
	pid.Kp = toCopy.Ki;
	pid.Ki = toCopy.Ki;
	pid.Kd = toCopy.Kd;
	pid.innerIntegralBand = toCopy.innerIntegralBand;
	pid.outerIntegralBand = toCopy.outerIntegralBand;
	pid.sigma = 0;
	pid.lastValue = 0;
	pid.lastTime = nPgmTime;
}

/**
 * calculate pid output
 *
 * @param pid  instance of PID structure
 * @param setPoint  set point of PID controller
 * @param processVariable  sensor/feedback value
 *
 * @return  output value of the control loop
 */
float
pidCalculate (PID pid, int setPoint, int processVariable) {
	float deltaTime = (nPgmTime - pid.lastTime)*0.001;
	pid.lastTime = nPgmTime;

	float deltaPV = 0;
	if(deltaTime > 0)
		deltaPV = (processVariable - pid.lastValue) / deltaTime;
	pid.lastValue = processVariable;

	float error = setPoint - processVariable;

	if(fabs(error) > pid.innerIntegralBand && fabs(error) < pid.outerIntegralBand)
		pid.sigma += error * deltaTime;

	if (fabs (error) > pid.outerIntegralBand)
		pid.sigma = 0;

	float output = error * pid.Kp
					+ pid.sigma * pid.Ki
					- deltaPV * pid.Kd;

	return output;
}

/**
 * calculate PID output while velocity control is active. The velocity set point will be subtracted from the time derivative of the error
 *
 * @param pid  the PID controller to use for the calculation
 * @param setPoint  the set point of the system
 * @param processVariable  the value of the feedback sensor in the system
 * @param velocitySet  the velocity set point of the system
 *
 * @return  the output value of the control loop
 */
float
pidCalculateWithVelocitySet (PID pid, int setPoint, int processVariable, int velocitySet) {
	float deltaTime = (nPgmTime - pid.lastTime)*0.001;
	pid.lastTime = nPgmTime;

	float deltaPV = 0;
	if(deltaTime > 0)
		deltaPV = (processVariable - pid.lastValue) / deltaTime + velocitySet;
	pid.lastValue = processVariable;

	float error = setPoint - processVariable;

	if(fabs(error) > pid.innerIntegralBand && fabs(error) < pid.outerIntegralBand)
		pid.sigma += error * deltaTime;

	if (fabs (error) > pid.outerIntegralBand)
		pid.sigma = 0;

	float output = error * pid.Kp
					+ pid.sigma * pid.Ki
					- deltaPV * pid.Kd;

	return output;
}

/**
 * calculate PID output for velocity control using feedforward instead of an error calculation, but still allowing for I and D components.
 *
 * @param pid  the PID controller to use for the calculation
 * @param setPoint  the set point of the system
 * @param processVariable  the value of the feedback sensor in the system
 *
 * @return  the output value of the control loop
 */
float
pidCalculateVelocity (PID pid, int setPoint, int processVariable) {
	float deltaTime = (nPgmTime - pid.lastTime)*0.001;
	pid.lastTime = nPgmTime;

	float deltaPV = 0;
	if(deltaTime > 0)
		deltaPV = (processVariable - pid.lastValue) / deltaTime;
	pid.lastValue = processVariable;

	float error = setPoint - processVariable;

	if(fabs(error) > pid.innerIntegralBand && fabs(error) < pid.outerIntegralBand)
		pid.sigma += error * deltaTime;

	if (fabs (error) > pid.outerIntegralBand)
		pid.sigma = 0;

	float output = setPoint * pid.Kp
					+ pid.sigma * pid.Ki
					- deltaPV * pid.Kd;

	return output;
}
#endif


/*--- Motion Planner Implementation ---*/


#ifndef NERD_MOTIONPLANNER
#define NERD_MOTIONPLANNER

#define MOVE_BUFFER_SIZE 10

#define SETTING_INACTIVE 0x0
#define SETTING_ACTIVE 0x1
#define SETTING_ACTIVEPOSITION 0x2
#define SETTING_MIRROR 0x3
#define SETTING_MIRROR_REVERSE 0x4

struct Move {
	long startTime;
	long timeLimit;
	float targetVelocity;
	char moveNotExecuted;
}

struct motionProfile {
	PID positionController;
	PID velocityController;

	float Kv;
	float Ka;
	float jerkLimit;
	void *sensor;
	int velocityFilter [5];
	int velocityRead;

	char profileSetting;
	short sensorIsFloat;
	short motorOutput;
	float positionOut;
	float lastSensorValue;
	long lastMeasureTime;
	long lastComputeTime;
	char cycleCounter;
	long moveStartTime;
	float aMax;
	float positionTarget;

	float velocityTarget;

	float positionSet; //position set point
	float velocitySet; //velocity set point
	float accelSet; //acceleration output
	float jerk; //rate of acceleration change

	float vMax; //max rate of system, in sensor units/second
	int accelTime; //time for velocity ramping
	int following; //motor port to mirror

	// + jerk
	long t1; //time to set jerk to 0
	// 0 jerk
	long t2; //time to set jerk to decelerate (accelSet -= jerk)
	// - jerk
	long t3; //time to set jerk to 0
	// 0 jerk

	Move moveBuffer [MOVE_BUFFER_SIZE]; //random access buffer for queueing move commands

	float cycleTime;
	int positionCycles; //amount of cycles to wait before new position update
};

task motionPlanner ();

void createMotionProfile (tMotor motorPort);

void profileSetSensor (tMotor motorPort, tSensors sensor); //
void profileSetSensorPtr (tMotor motorPort, void *sensor); //
void profileSetMaxVelocity (tMotor motorPort, float maxVelocity); //
void profileSetAccelerationGain (tMotor motorPort, float kA); //
void profileSetAccelerationTime (tMotor motorPort, int accelerationTime); //
void profileSetJerkRatio (tMotor motorPort, float jerkRatio); //
void profileSetSampleRate (tMotor motorPort, int samplesPerSecond); //
void profileSetPositionSampleRate (tMotor motorPort, int nVelocitySamples); //
void profileSetPositionController (tMotor motorPort, float Kp, float Ki, float Kd, float innerBand, float outerBand); //
void profileSetVelocityController (tMotor motorPort, float Kp, float Ki, float Kd, float innerBand, float outerBand); //
void profileSetVelocity (tMotor motorPort, float velocity); //
void profileSetMotorOutput (tMotor motorPort, short motorOutput); //
void profileSetMaster (tMotor motorPort, tMotor master, short reversed); //

void profileGoTo (tMotor motorPort, float position); //

float profileGetPositionSetpoint (tMotor motorPort);
float profileGetVelocitySetpoint (tMotor motorPort);
float profileGetAccelerationSetPoint (tMotor motorPort);
float profileGetVelocity (tMotor motorPort);
float profileGetPosition (tMotor motorPort);
short profileGetMotorOutput (tMotor motorPort);

void profileResetPosition (tMotor motorPort); //

//  because of ROBOTC not being true C we need to allocate space for profilers at compile time instead of instantiating them as we need them
//  ROBOTC is not a robot, nor is it C...
motionProfile profilerPool[10];
motionProfile* motorController [10];
motionProfile* uniqueControllers [10];

//sensor variable
int rawSensorValue [20];

float
getSensorValue (tMotor motorPort) {
	if (motorPort < port1 || motorPort > port10)
		return 0;

	if (motorController[motorPort] == NULL)
		return 0;

	motionProfile *profile = motorController[motorPort];
	void *var = profile->sensor;

	if (profile->sensorIsFloat) {
		float *f = var;
		return *f;
	} else {
		int *i = var;
		return (float)*i;
	}
}

float getSensorValue (motionProfile *profile) {
	if (profile == NULL)
		return 0;

	void *var = profile->sensor;

	if (profile->sensorIsFloat) {
		float *f = var;
		return *f;
	} else {
		int *i = var;
		return (float)*i;
	}
}

void
queueMove (motionProfile *profile, long startTime, float targetVelocity) {
	int i;

	for (i = 0; i < MOVE_BUFFER_SIZE; ++i) {
		if (profile->moveBuffer [i].moveNotExecuted == 0) {
			profile->moveBuffer [i].startTime = startTime;
			profile->moveBuffer [i].targetVelocity = targetVelocity;
			profile->moveBuffer [i].moveNotExecuted = 1;
			profile->moveBuffer [i].timeLimit = profile->accelTime;
			return;
		}
	}
}

void
queueMoveWithTimeLimit (motionProfile *profile, long startTime, float targetVelocity, long timeLimit) {
	int i;

	for (i = 0; i < MOVE_BUFFER_SIZE; ++i) {
		if (profile->moveBuffer [i].moveNotExecuted == 0) {
			profile->moveBuffer [i].startTime = startTime;
			profile->moveBuffer [i].targetVelocity = targetVelocity;
			profile->moveBuffer [i].moveNotExecuted = 1;
			profile->moveBuffer [i].timeLimit = timeLimit;
			return;
		}
	}
}

char
hasMoveQueued (motionProfile *profile) {
	int i;

	for (i = 0; i < MOVE_BUFFER_SIZE; ++i)
		if (profile->moveBuffer[i].moveNotExecuted == 1)
			return 1;

	return 0;
}

void
clearMoveQueue (motionProfile *profile) {
	int i;

	for (i = 0; i < MOVE_BUFFER_SIZE; ++i) {
		if (profile->moveBuffer [i].moveNotExecuted)
			profile->moveBuffer [i].moveNotExecuted = 0;
	}
}

/**
 * return a pointer to a ROBOTC sensor
 *
 * @param port  the sensor value to get a pointer to
 *
 * @return  the pointer to the sensor value
 */
int*
getSensorPointer (int port) {
	if (port < 0 || port > 19)
		return NULL;
	return &rawSensorValue [port];
}

void
createMotionProfile (tMotor motorPort) {
  if (motorPort < 0 || motorPort > 9)
		return;

	if (motorController [motorPort] != NULL)
		return;

  int i;
  motionProfile *controller;

  for (i = 0; i < 10; i++) {
    if (uniqueControllers[i] != NULL)
			continue;

		uniqueControllers [i] = &(profilerPool [i]);
		controller = uniqueControllers [i];
		break;
  }

  startTask (motionPlanner);

  motorController [motorPort] = controller;

  //autodetect encoder
  tSensors sensor = getEncoderForMotor (motorPort);
  if (sensor != -1)
   controller->sensor = getSensorPointer (sensor);

	controller->jerkLimit = 0.5;
	controller->velocityRead = 0;
	controller->profileSetting = SETTING_INACTIVE;
	controller->motorOutput = 0;

	if (sensor != -1)
	controller->lastSensorValue = getSensorValue(motorPort);

	controller->lastMeasureTime = nPgmTime;
	controller->lastComputeTime = nPgmTime;
	controller->vMax = 0;
  controller->positionOut = 0;
	controller->Ka = 0.0;

	controller->velocityFilter[0] = 0;
	controller->velocityFilter[1] = 0;
	controller->velocityFilter[2] = 0;
	controller->velocityFilter[3] = 0;
	controller->velocityFilter[4] = 0;

	controller->accelTime = 1000;
	controller->t1 = 0;
	controller->t2 = 0;
	controller->t3 = 0;

	controller->cycleTime = 20;
	controller->positionCycles = 4;

	pidInit (controller->positionController, 3.0, 0, 0, 30, 150);
	pidInit (controller->velocityController, 0, 0, 0, 50, 500);
}

void profileSetSensor (tMotor motorPort, tSensors sensor) {
  motorController[motorPort]->sensor = getSensorPointer (sensor);
}

void profileSetSensorPtr (tMotor motorPort, int *sensor) {
  motorController[motorPort]->sensor = sensor;
  motorController[motorPort]->sensorIsFloat = 0x0;
}

void profileSetSensorPtr (tMotor motorPort, float *sensor) {
  motorController[motorPort]->sensor = sensor;
  motorController[motorPort]->sensorIsFloat = 0x1;
}

void
profileSetMaxVelocity (tMotor motorPort, float maxVelocity) {
	if (motorController [motorPort] == NULL)
		return;

	motorController [motorPort]->vMax = maxVelocity;
	motorController [motorPort]->velocityController.Kp = 127.0 / maxVelocity;
}

void
profileSetSpeedLimit (tMotor motorPort, float speedLimit) {
	if (motorController [motorPort] == NULL)
		return;

	motorController [motorPort]->vMax = speedLimit;
}

void
profileSetAccelerationGain (tMotor motorPort, float kA) {
	if (motorController [motorPort] == NULL)
		return;

	motorController [motorPort]->Ka = kA;
}

void
profileSetAccelerationTime (tMotor motorPort, int accelerationTime) {
	if (motorController [motorPort] == NULL)
		return;

	motorController [motorPort]->accelTime = accelerationTime;
}

void
profileSetSampleRate (tMotor motorPort, int samplesPerSecond) {
	if (motorController [motorPort] == NULL)
		return;

  //in milliseconds
	motorController [motorPort]->cycleTime = (int) (1000.0 / samplesPerSecond);
}

void
profileSetPositionSampleRate (tMotor motorPort, int nVelocitySamples) {
	if (motorController [motorPort] == NULL)
		return;

	motorController [motorPort]->positionCycles = nVelocitySamples;
}

void
profileSetJerkRatio (tMotor motorPort, float jerkRatio) {
	if (motorController [motorPort] == NULL)
		return;

	if (jerkRatio > 1.0)
		jerkRatio = 1.0;
	else if (jerkRatio < 0.0)
		jerkRatio = 0.0;

	motorController [motorPort]->jerkLimit = jerkRatio;
}

float
profileGetPositionSetpoint (tMotor motorPort) {
  if (motorController [motorPort] == NULL)
		return -1;

  return motorController[motorPort]->positionSet;
}

float
profileGetVelocitySetpoint (tMotor motorPort) {
  if (motorController [motorPort] == NULL)
		return -1;

  return motorController[motorPort]->velocitySet;
}

float
profileGetAccelerationSetPoint (tMotor motorPort) {
  if (motorController [motorPort] == NULL)
		return -1;

  return motorController[motorPort]->accelSet;
}

float
profileGetVelocity (tMotor motorPort) {
    if (motorController [motorPort] == NULL)
		return -1;

  return motorController[motorPort]->velocityRead;
}

float
profileGetPosition (tMotor motorPort) {
  if (motorController [motorPort] == NULL)
		return -1;

  return getSensorValue (motorPort);
}

short
profileGetMotorOutput (tMotor motorPort) {
    if (motorController [motorPort] == NULL)
		return -1;

  return motorController[motorPort]->motorOutput;
}

short
profileMoveComplete (tMotor motorPort) {
	if (motorPort < port1 || motorPort > port10)
		return -1;

	if (motorController[motorPort] == NULL)
		return -1;

	return !hasMoveQueued (motorController[motorPort]);
}

/**
 * set the position PID controller for the specified motor's motion profile
 *
 * @param motorPort  the motor to update
 * @param Kp  proportional gain
 * @param Ki  integral gain
 * @param Kd  derivative gain
 * @param innerBand  the inner integral deadBand value
 * @param outerBand  the outer integral deadBand value
 */
void
profileSetPositionController (tMotor motorPort, float Kp, float Ki, float Kd, float innerBand, float outerBand) {
	if (motorController [motorPort] == NULL)
		return;

	motionProfile *profile = motorController [motorPort];
	pidInit (profile->positionController, Kp, Ki, Kd, innerBand, outerBand);
}

/**
 * set the velocity PID controller for the specified motor's motion profile
 *
 * @param motorPort  the motor to update
 * @param Kp  proportional gain
 * @param Ki  integral gain
 * @param Kd  derivative gain
 * @param innerBand  the inner integral deadBand value
 * @param outerBand  the outer integral deadBand value
 */
void
profileSetVelocityController (tMotor motorPort, float Kp, float Ki, float Kd, float innerBand, float outerBand) {
	if (motorController [motorPort] == NULL)
		return;

	motionProfile *profile = motorController [motorPort];
	pidInit (profile->velocityController, Kp, Ki, Kd, innerBand, outerBand);
}


void
profileSetMaster (tMotor motorPort, tMotor master, short reversed) {
	if (motorController [master] == NULL)
		return;

	if (motorController[motorPort] == NULL)
		createMotionProfile (motorPort);

  motorController[motorPort]->following = master;

  if (reversed) {
    motorController[motorPort]->profileSetting = SETTING_MIRROR_REVERSE;
  } else {
    motorController[motorPort]->profileSetting = SETTING_MIRROR;
  }
}

void profileResetPosition (tMotor motorPort) {
  if (motorController [motorPort] == NULL)
		return;

	if (motorController[motorPort]->sensor == NULL)
		return;

	if(motorController[motorPort]->sensorIsFloat) {
		float *ptr = motorController [motorPort]->sensor;
		*ptr = 0.0;
	} else {
		int *ptr = motorController [motorPort]->sensor;
		*ptr = 0;
	}
	motorController [motorPort]->lastSensorValue = 0 - motorController [motorPort]->velocityRead * motorController [motorPort]->cycleTime * 0.001;
}

/**
 * issue a move command to the specified position. This will currently do nothing if the move is considered to be a "short move", ie. the motor is unable to fully ramp up to max velocity during the move. Note that this is an absolute position command, so two consecutive moves to 4000 are not equivalent to a single move to 8000.
 *
 * @param motorPort  the motor to issue the move command to
 * @param position  the position to move to
 */
void
profileGoTo (tMotor motorPort, float position) {
	if (motorController [motorPort] == NULL)
		return;

	motionProfile *profile = motorController [motorPort];

  profile->positionSet = getSensorValue(motorPort);
	float distance = position - getSensorValue(motorPort);
	float initialVelocity = profile->velocityRead;
	float velocityError = sgn (distance) * profile->vMax - initialVelocity;
	float rampUpTime = fabs((sgn(distance) * profile->vMax - initialVelocity)/profile->vMax * profile->accelTime);
	float rampUpDist = 0.0005 * (rampUpTime-profile->accelTime) * initialVelocity + profile->accelTime / 2000.0 * profile->vMax * sgn (velocityError);
	float rampDownDist = profile->accelTime * profile->vMax / 2000.0 * sgn (velocityError);
	float cruiseTime = sgn (distance) * (distance - rampUpDist - rampDownDist) / profile->vMax * 1000.0;
	long decelTime = cruiseTime + rampUpTime + nPgmTime;

	clearMoveQueue(profile);
	queueMoveWithTimeLimit (profile, nPgmTime, sgn(distance) * profile->vMax, decelTime - nPgmTime);
	queueMoveWithTimeLimit (profile, decelTime, 0, decelTime - nPgmTime);
	profile->profileSetting = SETTING_ACTIVEPOSITION;
	profile->positionTarget = position;
}

/**
 * set a motor's output to a value from -127 to 127
 *
 * @param motorPort  the motor to set the output value of
 * @param output  output value to set
 */
void
profileSetMotorOutput (tMotor motorPort, short motorOutput) {
	if (motorController [motorPort] == NULL)
		return;

	if (motorOutput > 127)
		motorOutput = 127;
	if (motorOutput < -127)
		motorOutput = -127;

	clearMoveQueue (motorController[motorPort]);
	motorController[motorPort]->profileSetting = SETTING_INACTIVE;
	motorController[motorPort]->motorOutput = motorOutput;
	motorController[motorPort]->velocitySet = 0;
	motorController[motorPort]->positionTarget = 0;
	motorController[motorPort]->positionSet = 0;
}

/**
 * set a motor's velocity to the specified value
 *
 * @param motorPort  the motor to set
 * @param velocity  desired velocity
 */
void
profileSetVelocity (tMotor motorPort, float velocity) {
	if (motorController [motorPort] == NULL)
		return;

	motionProfile *profile = motorController[motorPort];
	profile->profileSetting = SETTING_ACTIVE;
	profile->positionSet = getSensorValue(profile);
	profile->positionTarget = getSensorValue (profile);

	clearMoveQueue (profile);
	queueMove (profile, nPgmTime, velocity);
}

void
updateMotors () {
	int i;

	for (i = 0; i < 10; ++i) {
		if (motorController [i] == NULL)
			continue;

    if (motorController[i]->profileSetting == SETTING_MIRROR)
			motorController[i]->motorOutput = motorController[motorController[i]->following]->motorOutput;

		if (motorController[i]->profileSetting == SETTING_MIRROR_REVERSE)
			motorController[i]->motorOutput = -motorController[motorController[i]->following]->motorOutput;

		if (motorController [i]->motorOutput > 127)
			motorController [i]->motorOutput = 127;
		else if (motorController [i]->motorOutput < -127)
			motorController [i]->motorOutput = -127;

		motor[i] = sgn(motorController [i]->motorOutput) * TrueSpeed[(int) fabs (motorController [i]->motorOutput)];
	}
}

void
measureVelocity (motionProfile *profile) {
	float deltaT = (nPgmTime - profile->lastMeasureTime)*0.001;
	float sensorV = getSensorValue(profile);

	//get sensor velocity, ticks per second
	float sensorRate = deltaT == 0 ? 0 : (sensorV - profile->lastSensorValue) / deltaT;

	for (int j = 4; j > 0; --j) {
		profile->velocityFilter [j] = profile->velocityFilter [j-1];
	}
	profile->velocityFilter [0] = sensorRate;

	sensorRate = profile->velocityFilter [0] * 0.5 + profile->velocityFilter [1] * 0.25 + profile->velocityFilter [2] * 0.125 + profile->velocityFilter [3] * 0.0625 + profile->velocityFilter [4] * 0.0625;
	profile->velocityRead = sensorRate;
	profile->lastSensorValue = sensorV;
}

void
startMove (motionProfile *profile, float targetVelocity, long timeLimit) {
	profile->velocityTarget = targetVelocity;
	float velocityError = targetVelocity - profile->velocityRead;
	long rampUpTime = fabs(velocityError / profile->vMax * profile->accelTime);

	if (rampUpTime > timeLimit)
		rampUpTime = timeLimit;

	long jerkTime = rampUpTime / 2.0 * profile->jerkLimit;

	if (rampUpTime < jerkTime) {
		jerkTime = 0;
	}

	if (rampUpTime != 0)
		profile->aMax = velocityError / (rampUpTime - jerkTime) * 1000.0;
	else
		profile->aMax = 0;

	if (jerkTime > 0)
		profile->jerk = profile->aMax / jerkTime * 1000.0;
	else
		profile->jerk = 0;

	profile->t1 = jerkTime;
	profile->t2 = rampUpTime - jerkTime;
	profile->t3 = rampUpTime;

	profile->moveStartTime = -1;
}

void
profileUpdate (motionProfile *profile) {
	if (profile->moveStartTime == -1)
		profile->moveStartTime = nPgmTime;

	float moveTime = nPgmTime - profile->moveStartTime;
	float deltaTime = nPgmTime - profile->lastComputeTime;

	if (profile->lastComputeTime < profile->moveStartTime) {
		deltaTime -= profile->moveStartTime - profile->lastMeasureTime;
		if (deltaTime < 0)
			deltaTime = 0;
	}
	profile->lastComputeTime = nPgmTime;

	if (nPgmTime < (unsigned long) (profile->moveStartTime + profile->t1)) { // t0
		profile->accelSet = profile->jerk * moveTime * 0.001;
	} else if (nPgmTime > (unsigned long) (profile->moveStartTime + profile->t1) && nPgmTime < (unsigned long) (profile->moveStartTime + profile->t2)) { // t1
		profile->accelSet = profile->aMax;
	} else if (nPgmTime > (unsigned long) (profile->moveStartTime + profile->t2) && nPgmTime < (unsigned long) (profile->moveStartTime + profile->t3)) { // t2
		profile->accelSet = profile->aMax - profile->jerk * (moveTime - profile->t2) * 0.001;
	} else if (nPgmTime > (unsigned long) (profile->moveStartTime + profile->t3)) { // t3
		profile->accelSet = 0;
		profile->velocitySet = profile->velocityTarget;
	}

	profile->velocitySet += profile->accelSet * deltaTime*0.001;
	profile->positionSet += profile->velocitySet * deltaTime*0.001;

	//do position PID if cycle includes it
	if (profile->cycleCounter % profile->positionCycles == 0) {
	 	profile->positionOut = pidCalculateWithVelocitySet (profile->positionController, profile->positionSet, getSensorValue (profile), profile->velocitySet);
	}
	profile->cycleCounter++;

	//do velocity PID
	float velocityOut = pidCalculateVelocity (profile->velocityController, profile->positionOut + profile->velocitySet, profile->velocityRead) + profile->accelSet * profile->Ka;
	//float velocityOut =  profile->velocitySet * profile->Kv + profile->accelSet * profile->Ka;//pidCalculate (profile->velocityController, profile->velocitySet + profile->positionOut, profile->velocityRead);

	//set motor PWM output
	profile->motorOutput = velocityOut;

	if (profile->motorOutput > 127)
		profile->motorOutput = 127;
	else if (profile->motorOutput < -127)
		profile->motorOutput = -127;
}

void
profileLog(tMotor motorPort) {
	if (motorPort < port1 || motorPort > port10)
		return;

	if (motorController[motorPort] == NULL)
		return;

	motionProfile *profile = motorController[motorPort];

	datalogDataGroupStart();
	datalogAddValue(0, profile->positionSet);
	datalogAddValue(1, getSensorValue (profile));
	datalogAddValue(2, profile->velocitySet);
	datalogAddValue(3, profile->velocityRead);
	datalogAddValue(4, profile->accelSet);
	datalogAddValue(5, profile->motorOutput);
	datalogDataGroupEnd();
}



task rawSensorMonitor () {
	int i;

	while (1) {
		for (i = 0; i < 20; ++i) {
			rawSensorValue [i] = SensorValue [i];
		}
	}

	abortTimeslice ();
}


task motionPlanner () {
	int i;

	startTask (rawSensorMonitor);

	while (1) {
		for (i = 0; i < 10; ++i) {
			if (uniqueControllers [i] == NULL)
				continue;

			motionProfile *profile = uniqueControllers [i];

			if (nPgmTime - profile->lastMeasureTime < profile->cycleTime) {
				continue;
			}

			measureVelocity (profile);
			profile->lastMeasureTime = nPgmTime;

			if (profile->profileSetting == SETTING_ACTIVE || profile->profileSetting == SETTING_ACTIVEPOSITION) {
				for (int j = 0; j < MOVE_BUFFER_SIZE; ++j) {
					if (profile->moveBuffer[j].moveNotExecuted && (unsigned long) (profile->moveBuffer[j].startTime) < nPgmTime) {
						startMove (profile, profile->moveBuffer[j].targetVelocity, profile->moveBuffer[j].timeLimit);
						profile->moveBuffer[j].moveNotExecuted = 0;

						break;
					}
				}

				if (profile->profileSetting == SETTING_ACTIVEPOSITION && (unsigned long) profile->t3 < nPgmTime && profile->velocitySet == 0 && !hasMoveQueued(profile))
					profile->positionSet = profile->positionTarget;

				profileUpdate (profile);
 			}
		}

		updateMotors ();
	}

	abortTimeslice ();
}

#endif
