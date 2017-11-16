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

//NOTE: dgt3 and/or dgt4 do not work

#pragma platform(VEX2)

#pragma competitionControl(Competition)

#include "Vex_Competition_Includes.c"
#include "\InTheZoneLibrary.c"

//for setLiftPos task
int desired;
float kp;
float BACK_KP = 1.2;
float MATCHLOAD_KP = 1.3;
float SCORE_KP = 0.8;

//for setClawUntilPos task
int desiredClaw;
int clawPower;
bool userControlClaw = true;

//Values increase as lift moves backwards
enum PotenValues {BACK = 1500, MATCHLOAD = 2000, SCORE = 4095, BACK_CLAW = 3700, MATCHLOAD_CLAW = 915};

bool reachedMobileGoal = false;

task setLiftPos() //for driver control
{
    bool ignore = false;
    if(desired == BACK && SensorValue[liftPoten]<BACK)
        ignore = true;
    int err = desired - SensorValue[liftPoten];
    int power = 127;

    while(abs(err)>200 &&  !ignore) //adjust power of motors while error is outide of certain range, then set power to 0
    {
        err = desired - SensorValue[liftPoten];
        power = (int) (err*127/4095*kp);
        setLiftPower(power);
        //writeDebugStreamLine("Poten: %d, Power: %d, Error: %d", SensorValue[liftPoten], power,err);
    }
    setLiftPower(0);

}

task setLiftPosAuton() //for auton: same as above, but holds lift in place and stops if mobile goal has been reached
{
    int err = desired - SensorValue[liftPoten];
    int power = 127;

    while(abs(err)>200 && !reachedMobileGoal) //adjust power of motors while error is outide of certain range, then set power to 0
    {
        err = desired - SensorValue[liftPoten];
        power = (int) (err*127/4095*kp);
        setLiftPower(power);
    }
    setLiftPower(-15);
    if(reachedMobileGoal)
        setLiftPower(0);
}

task setClawUntilPos()
{
    userControlClaw = false;
    setClawPower(clawPower);
    while(SensorValue[liftPoten]>desiredClaw){} //wait until lift goes past a certain point moving from score to back
    setClawPower(-clawPower);
    wait1Msec(10);
    setClawPower(0);
    userControlClaw = true;
}

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
    reachedMobileGoal = false; //will act as hard stop for lifting cone �?? when reachedMobileGoal is true, the lift will immediately drop

    //Go to mobile goal � Drop mobile base lift, lift cone, and drive straight
    setForkliftPower(1);
    desired = 3000;
    kp = 10;
    startTask(setLiftPosAuton); //lift up cone
    driveStraightAuton(1450,127); //drive to mobile goal

    //pick up goal
    reachedMobileGoal = true; //force cone lift to drop
    setForkliftPower(0); //pick up goal
    setLiftPower(0);
    wait1Msec(300);

    //drive back
    turnToPos(0);
    driveStraightAuton(-1000,127); //drive back -1000
    wait1Msec(300);

    //Score goal
    if(zone == 5)
    {
        //just turn around and drive straight
        turnToPos(-1800);
        driveStraightAuton(400,127);
    }
    else if(zone == 10)
    {
        //turn roughly parallel to white line, drive forward a bit, turn fully to face 10 pt zone, then drive straight
        turnToPos(-1315*minorSide);
        driveStraightAuton(300,127);
        //turnDeg(250);
        turnToPos(-2145*minorSide);
        driveStraightAuton(600,127);
    }
    wait1Msec(10);

    //Score cone and back away
    setClawPower(127);
    desired = BACK;
    kp = 0.9;
    startTask(setLiftPos);//lift up cone �?? possibly change this to not go back all the way (potentially wasting time in driver control)
    setForkliftPower(1);
    wait1Msec(500);
    setClawPower(0);
    driveStraightAuton(-600,127,1);
    writeDebugStreamLine("Time: %d", time1(T1));
}

task autonomous()
{
    string majorSide = "blue";
    int minorSide = 1; //left
    runBasicCompAuton(majorSide,minorSide,10);
    //runProgSkills(side);
}

task usercontrol()
{
    char direction = 1; //controls direction
    bool btnEightRightPressed = false; //tracks if button was pressed

    while(true)
    {
        if(vexRT[Btn7L]==1)
        {
            //actuallyDriveStraight(2);
            string side = "blue";
            runBasicCompAuton(side,-1,10);
            //driveStraightEncoders(3000, 127);
            //writeDebugStreamLine("Screw this button");
            //correctStraight(90);
            //driveStraightAuton(800,127);
        }
        //testing led
        if(SensorValue[liftPoten]<1000)
        {
            SensorValue[redLED] = true;
            SensorValue[yellowLED] = false;
            SensorValue[greenLED] = false;
        }
        else if(SensorValue[liftPoten]<2500)
        {
            SensorValue[redLED] = false;
            SensorValue[yellowLED] = true;
            SensorValue[greenLED] = false;
        }
        else
        {
            SensorValue[redLED] = false;
            SensorValue[yellowLED] = false;
            SensorValue[greenLED] = true;
        }
        //Buttons and Joysticks
        int  rightJoy = vexRT[Ch2];
        int  leftJoy = vexRT[Ch3];
        word rightTriggerUp = vexRT[Btn6U]; //for up lift
        word rightTriggerDown = vexRT[Btn6D]; //for down lift
        word leftTriggerUp = vexRT[Btn5U]; //for pincer close
        word leftTriggerDown = vexRT[Btn5D]; //for pincer open
        word btnEightUp = vexRT[Btn8U];
        word btnEightDown = vexRT[Btn8D]; //for lift to set point
        word btnSevenUp = vexRT[Btn7U]; //for lift to match loads
        word btnSevenDown = vexRT[Btn7D]; //for lift to match loads
        word btnEightRight = vexRT[Btn8R]; //for toggling reverse direction

        if(btnEightRight == 1 && !btnEightRightPressed){ //if button was pressed and was not already being pressed, change sign
            direction = -direction;
            btnEightRightPressed = true;
        }
        else if(btnEightRight == 0 && btnEightRightPressed) //if button is no longer being pressed, update bool
            btnEightRightPressed = false;

        //Drive Motors
        if(fabs(rightJoy) >= 15)
            if(direction==1)
                setRightMotors(rightJoy * 0.58);
            else
                setLeftMotors(rightJoy);
            else
                if(direction==1)
                    setRightMotors(0);
                else
                    setLeftMotors(0);

        if(fabs(leftJoy) >= 15)
            if(direction==1)
                setLeftMotors(leftJoy);
            else
                setRightMotors(leftJoy * 0.58);
            else
                if(direction==1)
                    setLeftMotors(0);
                else
                    setRightMotors(0);


        //Lift Motors
        if(rightTriggerUp == 1)
        {
            desired = SCORE;
            kp = SCORE_KP;
            startTask(setLiftPos);
        }
        else if(rightTriggerDown == 1)
        {
            desired = BACK;
            kp = BACK_KP;
            startTask(setLiftPos);
        }
        else if(btnSevenUp == 1)
        {
            desiredClaw = MATCHLOAD_CLAW;
            clawPower = 80;
            startTask(setClawUntilPos);

            desired = MATCHLOAD;
            kp = MATCHLOAD_KP;
            startTask(setLiftPos);
        }
        else if(btnSevenDown == 1)
        {
            desiredClaw = BACK_CLAW;
            clawPower = 80;
            startTask(setClawUntilPos);

            desired = BACK;
            kp = BACK_KP;
            startTask(setLiftPos);
        }

        //Mobile Goal Base Lifters
        if(btnEightUp == 1)
            setForkliftPower(0);
        else if(btnEightDown == 1)
            setForkliftPower(1);

        //pincer
        if(userControlClaw){
            if(leftTriggerDown == 1)
            {
                setClawPower(80);
            }
            else if(leftTriggerUp == 1)
            {
                setClawPower(-80);
            }
            else
            {
                setClawPower(0);
            }
        }
    }
}
