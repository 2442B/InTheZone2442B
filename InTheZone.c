#pragma config(Sensor, in1,    baseLiftPoten,  sensorPotentiometer)
#pragma config(Sensor, in2,    topLiftPoten,   sensorPotentiometer)
#pragma config(Sensor, in4,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  leftQuad,       sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  rightQuad,      sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  centerPiston,   sensorDigitalOut)
#pragma config(Sensor, dgtl8,  redLED,         sensorLEDtoVCC)
#pragma config(Sensor, dgtl9,  yellowLED,      sensorLEDtoVCC)
#pragma config(Sensor, dgtl10, greenLED,       sensorLEDtoVCC)
#pragma config(Sensor, dgtl11, leftPiston,     sensorDigitalOut)
#pragma config(Sensor, dgtl12, rightPiston,    sensorDigitalOut)
#pragma config(Motor,  port2,           topLift,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           driveLeftFront, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           driveLeftBack, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           driveRightFront, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           driveRightBack, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           baseLiftLeft,  tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           baseLiftRight, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           claw2,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port10,          claw,          tmotorVex393_HBridge, openLoop)
//NOTE: dgt3 and/or dgt4 do not work

#pragma platform(VEX2)

#pragma competitionControl(Competition)

#include "Vex_Competition_Includes.c"
#include "\InTheZoneLibrary.c"

void pre_auton()
{
	SensorValue[redLED] = 1;
	writeDebugStreamLine("begin gyro init");
	SensorType[in4] = sensorNone;
	wait1Msec(1000);
	SensorType[in4] = sensorGyro;
	wait1Msec(2000);
	SensorScale[in4] = 137;
	writeDebugStreamLine("finished gyro init %d", SensorScale[in4]);
	SensorValue[redLED] = 0;
	SensorValue[greenLED] = 1;
	//white line -- -1315


}
void runBasicCompAuton(string majorSide, int minorSide, int zone)
{
	//minorSide: 1 = left, -1 = right, majorSide parameter not used yet
	clearTimer(T1);
	reachedMobileGoal = false; //will act as hard stop for lifting cone â?? when reachedMobileGoal is true, the lift will immediately drop

	//Go to mobile goal â Drop mobile base lift, lift cone, and drive straight
	setForkliftPower(1);
	setTopLiftPos(3500,7,-15);
	driveStraight(1550,127); //drive to mobile goal

	//pick up goal
	reachedMobileGoal = true; //force cone lift to drop
	setForkliftPower(0); //pick up goal
	setTopLiftPower(0);
	wait1Msec(300);

	//drive back
	turnToPos(0);
	driveStraight(-1200,127); //drive back -1000
	wait1Msec(300);

	//Score goal
	if(zone == 5)
	{
		//just turn around and drive straight
		turnToPos(-1800*minorSide);
		driveStraight(400,127);
	}
	else if(zone == 10)
	{
		//turn roughly parallel to white line, drive forward a bit, turn fully to face 10 pt zone, then drive straight
		turnToPos(-1315*minorSide);
		driveStraight(400,127);
		//turnDeg(250);
		turnToPos(-2245*minorSide);
		driveStraight(750,127);
	}
	wait1Msec(10);

	//Score cone and back away
	setClawPower(127);
	setTopLiftPos(BACK_TOP,0.9); //lift up cone â?? possibly change this to not go back all the way (potentially wasting time in driver control)
	setForkliftPower(1);
	wait1Msec(500);
	setClawPower(0);
	driveStraight(-800,127,1);
	writeDebugStreamLine("Time: %d", time1(T1));
}

task autonomous()
{
	string majorSide = "blue";
	int minorSide = 1; //1 = left, -1 = right
	int zone = 10; //choose 5 or 10
	runBasicCompAuton(majorSide,minorSide,zone);
	//runProgSkills(side);
}

task usercontrol()
{
	bool coneUpPressed = false;
	bool coneDownPressed = false;
	bool coneZeroPressed = false;
	bool centerPushed = false; //Center piston pushed or naw

	while(true)
	{
		//if(vexRT[Btn7L]==1)
		//{
		//	string side = "blue";
		//	runBasicCompAuton(side,1,10);
		//}
		//if(vexRT[Btn7R]==1)
		//{
		//	string side = "blue";
		//	runProgSkills();
		//}
		//Buttons and Joysticks
		int  rightJoy = vexRT[Ch2];
		int  leftJoy = vexRT[Ch3];
		word rightTriggerUp = vexRT[Btn6U]; //for up top lift
		word rightTriggerDown = vexRT[Btn6D]; //for down top lift
		word leftTriggerUp = vexRT[Btn5U]; //for pincer close
		word leftTriggerDown = vexRT[Btn5D]; //for pincer open
		word btnEightUp = vexRT[Btn8U]; //for up base lift
		word btnEightDown = vexRT[Btn8D]; //for down base lift
		word btnSevenUp = vexRT[Btn7U]; //forklift up
		word btnSevenDown = vexRT[Btn7D]; //forklift down
		word btnSevenRight = vexRT[Btn7R]; //Pelvic thrust
		word btnEightLeft = vexRT[Btn8L]; //auto score
		word btnEightRight = vexRT[Btn8R]; //auto back
		word secondBtnSevenUp = vexRT[Btn7UXmtr2]; //+1 to cone count
		word secondBtnSevenDown = vexRT[Btn7DXmtr2]; //-1 to cone count
		word secondBtnSevenLeft = vexRT[Btn7LXmtr2]; //zero cone count

		//Drive Motors
		if(fabs(rightJoy) >= 15)
			setRightMotors(rightJoy);
		else
			setRightMotors(0);

		if(fabs(leftJoy) >= 15)
			setLeftMotors(leftJoy);
		else
			setLeftMotors(0);


		//Lift Motors
		if(rightTriggerUp == 1)
		{
			//setTopLiftPower(127);
			setTopLiftPos(SCORE_TOP,SCORE_KP_TOP);
		}
		else if(rightTriggerDown == 1)
		{
			//setTopLiftPower(-80);
			setTopLiftPos(BACK_TOP,BACK_KP_TOP);
		}
		/*
		else if(btnSevenUp == 1)
		{
			setClawUntilPos(MATCHLOAD_CLAW,80);
			setTopLiftPos(MATCHLOAD,MATCHLOAD_KP);
		}
		else if(btnSevenDown == 1)
		{
			setClawUntilPos(BACK_CLAW,80);
			setTopLiftPos(BACK,BACK_KP);
		}
		*/
		//else
		//{
		//	setTopLiftPower(0);
		//}

		if(btnEightUp == 1)
		{
			setBaseLiftPower(-127);
		}
		else if(btnEightDown == 1)
		{
			setBaseLiftPower(80);
		}
		else
		{
			setBaseLiftPower(0);
		}

		//Mobile Goal Base Lifters
		if(btnSevenUp == 1)
		{
			setForkliftPower(1);
			/*
			if(centerPushed)
			{
				SensorValue(centerPiston) = 0;
				centerPushed = false;
			}
			*/
		}
		else if(btnSevenDown == 1)
			setForkliftPower(0);
	/*
			if(centerPushed)
			{
				SensorValue(centerPiston) = 0;
				centerPushed = false;
			}
			*/ //might get caught while withdrawing? task? refuse to go up or down while extended? good idea poor execution?

		if(btnSevenRight == 1 && !centerPushed)
		{
			SensorValue(centerPiston) = 1;
			centerPushed = true;
		}
		else if(btnSevenRight == 1 && centerPushed)
		{
			SensorValue(centerPiston) = 0;
			centerPushed = false;
		}


		//claw
		if(userControlClaw){
			if(leftTriggerDown == 1)
				setClawPower(80); //open claw
			else if(leftTriggerUp == 1)
				setClawPower(-80); //close claw
			else
				setClawPower(0);
		}

		//testing up and down
		if(btnEightLeft == 1){autoBack();}
		else if(btnEightRight == 1){autoScore(); writeDebugStreamLine("Cones Stacked: %d", conesStacked);}

		//cone count
		if(secondBtnSevenUp == 1 && !coneUpPressed) //if button is now pressed, update cones and update bool to reflect button pressed
		{
			conesStacked++;
			coneUpPressed = true;
		}
		else if(secondBtnSevenUp == 0 && coneUpPressed) //if button is no longer pressed, update bool to reflect lack of press
		{
			coneUpPressed = true;
		}

		if(secondBtnSevenDown == 1 && !coneDownPressed) //if button is now pressed, update cones and update bool to reflect button pressed
		{
			conesStacked--;
			coneDownPressed = true;
		}
		else if(secondBtnSevenDown == 0 && coneDownPressed) //if button is no longer pressed, update bool to reflect lack of press
		{
			coneDownPressed = true;
		}

		if(secondBtnSevenLeft == 1 && !coneZeroPressed) //if button is now pressed, update cones and update bool to reflect button pressed
		{
			conesStacked = 0;
			coneZeroPressed = true;
		}
		else if(secondBtnSevenLeft == 0 && coneZeroPressed) //if button is no longer pressed, update bool to reflect lack of press
		{
			coneZeroPressed = true;
		}
	}
}
