#pragma config(Sensor, in3,    secondBattery,  sensorAnalog)
#pragma config(Sensor, in4,    gyro,           sensorGyro)
#pragma config(Sensor, in7,    topLiftPoten,   sensorPotentiometer)
#pragma config(Sensor, in8,    baseLiftPoten,  sensorPotentiometer)
#pragma config(Sensor, dgtl1,  rightQuad,      sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  forkliftButton, sensorDigitalIn)
#pragma config(Sensor, dgtl6,  greenLED,       sensorLEDtoVCC)
#pragma config(Sensor, dgtl7,  sideToggle,     sensorDigitalIn)
#pragma config(Sensor, dgtl8,  minorZoneToggle, sensorDigitalIn)
#pragma config(Sensor, dgtl9,  majorZoneToggle, sensorDigitalIn)
#pragma config(Sensor, dgtl11, leftQuad,       sensorQuadEncoder)
#pragma config(Motor,  port2,           rollers,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           driveLeftFront, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           driveLeftBack, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           driveRightBack, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           driveRightFront, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           mobileLiftLeft, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           mobileLiftRight, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           baseLift,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          fourBar,       tmotorVex393_HBridge, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//NOTE: dgt3 and/or dgt4 do not work

#pragma systemFile

////GLOBAL VARIABLES////
//Poten Values For Lift -- Values increase as lift moves backwards
enum ForkliftPos {FORKLIFT_UP=1,FORKLIFT_DOWN=-1};
enum PotenValuesTop {BACK_TOP = 1200, UPRIGHT_TOP = 2008, MATCHLOAD_TOP = 580, SCORE_TOP = 3750, FLAT_TOP=1900};
enum PotenValuesClaw {BACK_CLAW = 3700, MATCHLOAD_CLAW = 750};
enum PotenValuesBase {BACK_BASE = 1080, MATCHLOAD_BASE = 700, HIGHEST_BASE =  0}; //values increase as lift moves down
int basicTopPositions[3] = {1270, 2000, 3810};
int basicTopKp[3] = {0.3,0.3,0.3};
int topLiftPositions[12] = {3700,2600,2775,2600,2600,2600,2600,2600,2600,2600,2600,2600};
int baseLiftPositions[12] = {3600,3550,3400,3300,3100,3000,2900,3400,3300,3250,2695,2525};
int secondBaseLiftPositions[12] = {0,3450,3400,3300,3100,3000,2900,3400,3300,3250,2695,2525};

/*base
back = 3980
highest = 2423

topLift:
back = 355
up = 2608
scoreAll = 4095 (deadzone after)
*/
float BACK_KP_TOP = 10;
float MATCHLOAD_KP_TOP = 2;
float SCORE_KP_TOP = 10;
float FLAT_KP_TOP = 10;
float BACK_KP_BASE = 10;
float SCORE_KP_BASE = 250	;

float MATCHLOAD_KP_BASE = 10;
int ERR_MARGIN = 50;

//for forkliftPos task/method
float forkliftPos = FORKLIFT_UP;

//for correctStraight task / driveStraight method
float rightPowerAdjustment = 0;
float leftPowerAdjustment = 0;
float theta = 0;

//for setTopLiftPos task / setTopLiftPos method
int desiredTop;
int powAfterTop;
float kpTop = 1;
bool reachedMobileGoal = false;
int topLiftTimeLimit = 2500;

//for setBaseLiftPos task / setBaseLiftPos method
int desiredBase;
int powAfterBase;
float kpBase;
int conesStacked = 0;

//for setClawUntilPos task / setClawUntilPos
int desiredClaw;
int clawPower;
bool userControlClaw = true;
bool userControlBase = true;

//for setForliftPos
int forkliftTime = 1800;

/////BASIC MOTOR METHODS/////
void setLeftMotors(int power)
{
	motor[driveLeftBack] = power;
	motor[driveLeftFront] = power;
}

void setRightMotors(int power)
{
	motor[driveRightBack] = power;
	motor[driveRightFront] = power;
}
void setAllDriveMotors(int power)
{
	setLeftMotors(power);
	setRightMotors(power);
}

void setTopLiftPower(int power)
{
	motor[fourBar] = power;
}


void setBaseLiftPower(int power)
{
	motor[baseLift] = power;
}


void setForkliftPower(int power)
{
	motor[mobileLiftLeft] = power;
	motor[mobileLiftRight]  = power;
}

void setClawPower(int power)
{
	motor[rollers] = power;

}

void basicSlewControlDrive(int power)
{
	int currentPower = 0;
	while(currentPower<power)
	{
		currentPower += 15;
		if(currentPower > power){currentPower = power;}
		setAllDriveMotors(currentPower);
		wait1Msec(40);
	}
	writeDebugStreamLine("slew finished");
}

/////////TASKS/////////
task correctStraight()
{
	rightPowerAdjustment = 0;
	leftPowerAdjustment = 0;
	int err = theta - SensorValue[gyro];
	int oldErr = err;
	int power;
	int deriv;
	int totalErr = err;
	int integral = totalErr;
	while(1)
	{
		err = theta - SensorValue[gyro];
		deriv = (err-oldErr); //if error is increasing, apply more power (compensate for less momentum). else, apply less power
		//if(fabs(err)<10)
			//integral = 0
		//else
			integral = totalErr * 0.1;
		power = err*1.8 + deriv*1.0 + integral;
		if(power>0)
		{
			rightPowerAdjustment = 0;
			leftPowerAdjustment = -power;
		}
		else
		{
			rightPowerAdjustment = power;
			leftPowerAdjustment = 0;
		}
		oldErr = err;
		totalErr += err;
		writeDebugStreamLine("Drive, Err: %d, Deriv: %d, TotalErr: %d, Integral: %d, Power: %d", err,deriv,totalErr,integral,power);
		wait1Msec(50);
	}
}

/*
task setTopLiftPosTask() //reachedMobileGoal is only used in auton to stop and hold lift in place if robot reaches goal unexpectedly early
{
bool ignore = false;
if((desiredTop == BACK_TOP && SensorValue[topLiftPoten]<BACK_TOP) || reachedMobileGoal)
ignore = true;
int err = desiredTop - SensorValue[topLiftPoten];
int power = 127;

while(abs(err)>25 &&  !ignore) //adjust power of motors while error is outide of certain range, then set power to 0
{
err = desiredTop - SensorValue[topLiftPoten];
power = (int) (err*127/4095*kpTop); //HAD MINUS SIGN
setTopLiftPower(power);
//writeDebugStreamLine("Desired: %d, Poten: %d, Power: %d, Error: %d", desiredTop, SensorValue[topLiftPoten], power,err);
wait1Msec(50);
}
//writeDebugStreamLine("Moving to powAfter");
setTopLiftPower(powAfterTop);
if(reachedMobileGoal)
setTopLiftPower(0);
}
*/

task holdTopLiftPosTask()
{
	int err = desiredTop - SensorValue[topLiftPoten];
	int holdTopDeriv = 0;
	int power = 127;
	int holdTopTotal = 0;
	int holdTopPrevious = 0;


	while(1) //adjust power of motors while error is outide of certain range, then set power to 0
	{
		err = desiredTop - SensorValue[topLiftPoten];
		holdTopDeriv = err - holdTopPrevious;
		//writeDebugStreamLine("4bar kp: %f"
		if(desiredTop == basicTopPositions[1])
		{
			power = (int) (err*0.2 + holdTopDeriv*0.1 + holdTopTotal*0);
		}
		else{
		power = (int) (err*0.4 + holdTopDeriv*0.1 + holdTopTotal*0); //USING KP INSTEAD OF MANUAL 0.3 DOES NOT WORK - NEED TO DEBUG
	}
		//writeDebugStreamLine("Desired: %d, Poten: %d, Power: %d, Error: %d", desiredTop, SensorValue[topLiftPoten], power,err);

		holdTopPrevious = err;
		holdTopTotal += err;
		wait1Msec(50);

		if(fabs(power)>30 && ((fabs(err)>20) || (desiredTop == basicTopPositions[1]) || !(desiredTop==basicTopPositions[0] && sensorValue[topLiftPoten]<basicTopPositions[0])))
		{
			setTopLiftPower(power);
		}
		else
		{
			setTopLiftPower(0);
		}
	}
}

task setBaseLiftPosTask()
{
	userControlBase = false;
	int err = desiredBase - SensorValue[baseLiftPoten];
	int power = 127;
	while(abs(err)>ERR_MARGIN) //adjust power of motors while error is outide of certain range, then set power to 0
	{
		err = desiredBase - SensorValue[baseLiftPoten];
		power = (int) (err*127/2000*kpBase);
		setBaseLiftPower(power);
		//writeDebugStreamLine("Poten: %d, Power: %d, Error: %d", SensorValue[baseLiftPoten], power,err);
		wait1Msec(50);
	}
	setBaseLiftPower(powAfterBase);

}

task holdBaseLiftPosTask()
{
	userControlBase = false;
	int power;

	//proportional
	int errBase = desiredBase - SensorValue[baseLiftPoten];

	//deriv
	int derivBase = 0;
	int previousErrBase = 0;
	int previousDerivBase = 0;

	//integeral
	int totalErrBase = 0;

	clearTimer(T3);

	while(time1[T3] < topLiftTimeLimit) //adjust power of motors while error is outide of certain range, then set power to 0
	{
		errBase = desiredBase - SensorValue[baseLiftPoten];
		derivBase = errBase - previousErrBase;

		//if(sgn(derivBase)!=sgn(previousDerivBase))
		//{
		//    writeDebugStreamLine("Switch at err: %d, t: %f", errBase, time1(T3));
		//    clearTimer(T3);
		//}
		datalogDataGroupStart();
		datalogAddValue(0,errBase);
		datalogDataGroupEnd();

		power = (int) (errBase*0.18 + derivBase*0.3 +totalErrBase*0);
		setBaseLiftPower(power);

		previousErrBase = errBase;
		totalErrBase += errBase;

		wait1Msec(50);
	}
}

task setClawUntilPosTask()
{
	userControlClaw = false;
	setClawPower(clawPower);
	while(SensorValue[topLiftPoten]>desiredClaw){} //wait until lift goes past a certain point moving from score to back
	setClawPower(-clawPower);
	wait1Msec(10);
	setClawPower(0);
	userControlClaw = true;
}

task setForkliftPosTask()
{
	clearTimer(T4);
	setForkliftPower(forkliftPos*80);
	while((SensorValue(forkliftButton) == 1 || forkliftPos==FORKLIFT_DOWN) && time1(T4)<forkliftTime){wait1Msec(20);}
	setForkliftPower(0);
}

///////COMPLEX METHODS: a+bi///////
void setForkliftPos(int aForkPos, int aForkliftTime = 1800)
{
	forkliftPos = aForkPos;
	forkliftTime = aForkliftTime;
	startTask(setForkliftPosTask);
}

void driveStraight(int dest, int basePower = 127, float leftMultiplier = 0.9, int timeLimitingInstead = 0, bool hold=false) //uses correctStraight task (with gyro) to dive straight
{
	if(timeLimitingInstead != 0)
	{
		clearTimer(T2);
	}
	theta = SensorValue[gyro];
	//reset encoders
	SensorValue[leftQuad] = 0;
	SensorValue[rightQuad] = 0;
	basicSlewControlDrive(basePower*sgn(dest));

	int err = dest;
	int power = 127;
	rightPowerAdjustment = 0;
	leftPowerAdjustment = 0;
	startTask(correctStraight);
	//writeDebugStreamLine("err: %d, power: %d sdfdgdsgfgfsggffs",err,power);
	while(fabs(err)>20 && fabs(dest - (SensorValue[leftQuad]))>20 && !(timeLimitingInstead !=0 && time1(T2)>timeLimitingInstead))
	{
		err = dest - SensorValue[leftQuad];
		power = basePower*sgn(err);
		setRightMotors((int)(power + rightPowerAdjustment));
		setLeftMotors((int) ((power + leftPowerAdjustment)*leftMultiplier));
		//writeDebugStreamLine("RightMotor: %d, LeftMotors: %d", (int)(power + rightPowerAdjustment), (int) ((power + leftPowerAdjustment)*leftMultiplier));
		//writeDebugStreamLine("motors set");
		wait1Msec(50);
		//writeDebugStreamLine("RightAdjustment: %d, LeftAdjustment: %d", rightPowerAdjustment, leftPowerAdjustment);
		//writeDebugStreamLine("Drive! err: %d, power: %d, rpower: %d",err,power,(int)(power + rightPowerAdjustment));
	}
	stopTask(correctStraight);
	setAllDriveMotors(0);
}

void turnToPos(int pos,bool withMobileGoal=true,int timeLimit = 2500)
{
	clearTimer(T4);
	int err = pos - SensorValue[gyro];
	int power;
	int errTerm;
	int derivTerm;
	int intTerm;
	int oldErrTerm = 0;
	int totalErrTerm = 0;
	while(fabs(err) > 10 && time1(T4)<timeLimit)
	{
		err = pos - SensorValue[gyro];
		if(!withMobileGoal)
		{
			errTerm = err*.5;
			derivTerm = (err - oldErrTerm)*1.3;
			intTerm = 0;//totalErrTerm*0.004;
		}
		else
		{
			errTerm = err*0.9;
			derivTerm = (err - oldErrTerm)*1;
			intTerm = totalErrTerm*0;
		}


		power = errTerm+derivTerm+intTerm;

		setRightMotors(power);
		setLeftMotors(-power);
		//writeDebugStreamLine("Turning, Err: %d, power: %d", err, power);

		oldErrTerm = err;
		totalErrTerm += err;
		wait1Msec(50);
	}
	setAllDriveMotors(0);
	//writeDebugStreamLine("Cleared turntopos, t4: %i", time1[T4]);
}

/*
void setTopLiftPos(int aDesired, float aKp, int aPowAfter = 0)
{
reachedMobileGoal = false;
desiredTop = aDesired;
kpTop = aKp;
powAfterTop = aPowAfter;
startTask(setTopLiftPosTask);
}*/

void holdTopLiftPos(int aDesired, float aKp, int aPowAfter = 0)
{
	reachedMobileGoal = false;
	desiredTop = aDesired;
	kpTop = aKp;
	writeDebugStreamLine("aKp is %f",aKp);
	powAfterTop = aPowAfter;
	startTask(holdTopLiftPosTask);
}

void setTopLiftPos(int aDesired, float aKp, int aPowAfter = 0, int timeLimit = 2500)
{
	reachedMobileGoal = false;
	desiredTop = aDesired;
	kpTop = aKp;
	writeDebugStreamLine("aKp is %f",aKp);
	powAfterTop = aPowAfter;
	startTask(holdTopLiftPosTask);
}

void setBaseLiftPos(int aDesired, float aKp, int aPowAfter = 0)
{
	reachedMobileGoal = false;
	desiredBase = aDesired;
	kpBase = aKp;
	powAfterBase = aPowAfter;
	startTask(setBaseLiftPosTask);
}

void holdBaseLiftPos(int aDesired)
{
	reachedMobileGoal = false;
	desiredBase = aDesired;
	startTask(holdBaseLiftPosTask);
}

void setClawUntilPos(int aDesiredClaw, int aClawPower)
{
	desiredClaw = aDesiredClaw;
	clawPower = aClawPower;
	startTask(setClawUntilPosTask);
}

/////MORE COMPLEX TASKS - AUTOSTACKING///
task autoScoreTask()
{
	int topLiftStart = 1*topLiftPositions[conesStacked]; //poten units of base lift corresponing to top lift swing-around time, assuming poten increases towards score (increasing distance == increasing time alloted)
	//writeDebugStreamLine(" baseLiftPosDesired: %d", 	baseLiftPositions[conesStacked]);
	setBaseLiftPos(baseLiftPositions[conesStacked],SCORE_KP_BASE);
	//writeDebugStreamLine("must be less than this level: %d", baseLiftPositions[conesStacked] - topLiftStart - ERR_MARGIN);
	while(SensorValue[baseLiftPoten] > baseLiftPositions[conesStacked] + topLiftStart + ERR_MARGIN){wait1Msec(20);} //assuming poten decreases towards up
	wait1Msec(20);
	writeDebugStreamLine("first while loop in score");
	while(SensorValue[baseLiftPoten] > baseLiftPositions[conesStacked] + topLiftStart + ERR_MARGIN){wait1Msec(20);}
	writeDebugStreamLine("second while in score");
	holdTopLiftPos(topLiftPositions[conesStacked],SCORE_KP_TOP);
	while(SensorValue[topLiftPoten] < topLiftPositions[conesStacked] - ERR_MARGIN){wait1Msec(20);}
	writeDebugStreamLine("third while in score");
	if(secondBaseLiftPositions[conesStacked] > 0 )
		setBaseLiftPos(secondBaseLiftPositions[conesStacked],SCORE_KP_BASE);
	conesStacked++;
	writeDebugStreamLine("last while in score");
}

task autoBackTask()
{
	userControlClaw = false;
	if(SensorValue[topLiftPoten]>UPRIGHT_TOP)
		setClawPower(127); //open claw
	holdTopLiftPos(BACK_TOP,BACK_KP_TOP);
	while(SensorValue[topLiftPoten]>UPRIGHT_TOP){}
	setBaseLiftPos(BACK_BASE,BACK_KP_BASE,0);
	setClawPower(0); //close claw
}

task autoStackTask()
{
	int thisConeStack = conesStacked;
	startTask(autoScoreTask);
	while(SensorValue[topLiftPoten] < topLiftPositions[conesStacked] - ERR_MARGIN || SensorValue[baseLiftPoten] > baseLiftPositions[conesStacked] + ERR_MARGIN){wait1Msec(50);}
	writeDebugStreamLine("after 1st while in autoStack");
	//while(SensorValue[topLiftPoten] < topLiftPositions[conesStacked] - ERR_MARGIN || (secondBaseLiftPositions[conesStacked] > 0 && SensorValue[baseLiftPoten] > secondBaseLiftPositions[conesStacked] + ERR_MARGIN)){wait1Msec(50);}
	wait1Msec(200);
	writeDebugStreamLine("Starting autoBack task");
	startTask(autoBackTask);
}

////AND CORRESPONDING MORE COMPLEX METHODS/////
void autoScore()
{
	startTask(autoScoreTask);
	wait1Msec(3000);
}

void autoBack()
{
	startTask(autoBackTask);
}

void autoStack()
{
	startTask(autoStackTask);
}
