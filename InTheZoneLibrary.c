#pragma config(Sensor, in1,    leftClawPoten,  sensorPotentiometer)
#pragma config(Sensor, in2,    liftPoten,      sensorPotentiometer)
#pragma config(Sensor, in3,    rightClawPoten, sensorPotentiometer)
#pragma config(Sensor, in4,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  leftQuad,       sensorQuadEncoder)
#pragma config(Sensor, dgtl6,  rightQuad,      sensorQuadEncoder)
#pragma config(Sensor, dgtl8,  redLED,         sensorLEDtoVCC)
#pragma config(Sensor, dgtl9,  yellowLED,      sensorLEDtoVCC)
#pragma config(Sensor, dgtl10, greenLED,       sensorLEDtoVCC)
#pragma config(Sensor, dgtl11, leftPiston,     sensorDigitalOut)
#pragma config(Sensor, dgtl12, rightPiston,    sensorDigitalOut)
#pragma config(Motor,  port1,           claw2,         tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           test,          tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           driveLeftFront, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           driveLeftBack, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           driveRightFront, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           driveRightBack, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           liftLeft,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           liftRight,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           forklift,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          claw,          tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma systemFile

float rightPowerAdjustment = 0;
float leftPowerAdjustment = 0;
float theta = 0;

task correctStraight() //UNUSED METHOD/FOR TESTING GYRO
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
		deriv = (err-oldErr)*0.5; //if error is increasing, apply more power (compensate for less momentum). else, apply more power
		integral = totalErr * 0.03;
		power = err*0.5 + deriv + integral;
		rightPowerAdjustment = power;
		leftPowerAdjustment = -power;
		oldErr = err;
		totalErr += err;
		//writeDebugStreamLine("Err: %d, Deriv: %d, TotalErr: %d, Integral: %d, Power: %d", err,deriv,totalErr,integral,power);
		wait1Msec(50);
	}
}

void setLeftMotors(int power)
{
	motor[driveLeftFront] = power;
	motor[driveLeftBack] = power;
}

void setRightMotors(int power)
{
	motor[driveRightFront] = power;
	motor[driveRightBack] = power;
}
void setAllDriveMotors(int power)
{
	setLeftMotors(power);
	setRightMotors(power);
}

void setLiftPower(int power)
{
	motor[liftLeft] = power;
	motor[liftRight] = power;
}

void setForkliftPower(int power)
{
	//motor[forklift] = power;
	SensorValue[rightPiston] = power;
	SensorValue[leftPiston]  = power;
}

void setClawPower(int power)
{
	motor[claw] = power;
	motor[claw2] = power;

}

void turnDeg(int angle) //TO BE REMOVED AFTER EVERYTHING IS CONVERTED TO TURNTOPOS()
{
	SensorValue[rightQuad] = 0;
	SensorValue[leftQuad] = 0;
	int adjustedAngle = angle*385/360;
	int err = adjustedAngle;
	int power;
	while(abs(err)>20)
	{
		err = adjustedAngle - SensorValue[leftQuad];
		power=-127;
		writeDebugStreamLine("err: %d",err);
		//power = err*127/ajustedAngle + 10;
		setRightMotors(power);
		setLeftMotors(-power);
	}
	setRightMotors(sgn(angle) * -63);
	setLeftMotors(sgn(angle) * 63);
	wait10Msec(3);
	setAllDriveMotors(0);
}

void driveStraightAuton(int dest, int basePower, float rightMultiplier = 0.6)
{
	theta = SensorValue[gyro];
	SensorValue[leftQuad] = 0;
	SensorValue[rightQuad] = 0;
	int err = dest;
	int power = 127;
	startTask(correctStraight);
			writeDebugStreamLine("err: %d, power: %d sdfdgdsgfgfsggffs",err,power);
	while(fabs(err)>20 && fabs(dest - (-1*SensorValue[rightQuad]))>20)
	{
		err = dest - SensorValue[leftQuad];
		power = basePower*sgn(err);
		setRightMotors((int)(power*rightMultiplier + rightPowerAdjustment));
		setLeftMotors((int) (power+leftPowerAdjustment));
		//writeDebugStreamLine("RightAdjustment: %d, LeftAdjustment: %d", rightPowerAdjustment, leftPowerAdjustment);
		//writeDebugStreamLine("err: %d, power: %d, rpower: %d",err,power,(int)(power*rightMultiplier + rightPowerAdjustment));
	}
	stopTask(correctStraight);
	setAllDriveMotors(0);
}

void driveStraightEncoders(int dest, int basePower)
{
	SensorValue[rightQuad] = 0;
	SensorValue[leftQuad] = 0;
	int leftError = dest - SensorValue(leftQuad);
	float leftConst = 0.5;
	int rightError = dest - (-1 * SensorValue(rightQuad));
	float rightConst = 0.1;
	while(fabs(leftError * leftConst) > 0 && fabs(rightError * rightConst * 0.58) > 0)
	{
		leftError = dest - SensorValue(leftQuad);
		rightError = dest - (-1 * SensorValue(rightQuad));
		setLeftMotors((int)(leftConst * leftError));
		setRightMotors((int)(rightConst * rightError));
	}
	writeDebugStreamLine("Did stuff. Are you proud yet?");
	//clean
	setRightMotors(0);
	setLeftMotors(0);
}


void turnToPos(int pos)
{
	clearTimer(T4);
	int err = pos - SensorValue[gyro];
	int power;
	while(fabs(err) > 80 && time1(T4)<2500)
	{
		err = pos - SensorValue[gyro];
		power = 127*err*0.004+10;
		setRightMotors(power);
		setLeftMotors(-power);
		wait1Msec(50);
	}
}
//void actuallyDriveStraight(int time)
//{
//	clearTimer(T1);
//	SensorValue[rightQuad] = 0;
//	SensorValue[leftQuad]= 0;
//	int masterPower = -100;
//	int adjustedPower =masterPower*1.17;
//	setRightMotors(adjustedPower);
//	setLeftMotors(masterPower);
//	int err;
//	int left;
//	int right;
//	while(time100(T1)<time*10)
//	{
//		int left = SensorValue[leftQuad];
//		right = - SensorValue[rightQuad];
//		err = left - right;
//		adjustedPower += 0.4*err;
//		writeDebugStreamLine("Left Speed: %d, Right Speed: %d, Error: %d, AdjustedPower: %d",left,right,err,adjustedPower);
//		setRightMotors(adjustedPower);

//		SensorValue[rightQuad] = 0;
//		SensorValue[leftQuad] = 0;

//		wait1Msec(50);
//	}
//}
