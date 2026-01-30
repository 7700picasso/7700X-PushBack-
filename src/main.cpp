/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student2                                                  */
/*    Created:      10/9/2025, 5:00:24 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
motor RF (PORT17, ratio6_1, false);
motor RM (PORT20, ratio6_1, false);
motor RB (PORT18, ratio6_1, false);
motor LF (PORT15, ratio6_1, true);
motor LM (PORT16, ratio6_1, true);
motor LB (PORT13, ratio6_1, true);
motor outake1 (PORT9, ratio6_1, 1);//red sprocket
motor outake2 (PORT19, ratio6_1, true);//green sprocket
motor conveyor (PORT14, ratio6_1, false);
inertial gyr (PORT1);
digital_out ScrapaparerDescorerere = digital_out(Brain.ThreeWirePort.A);
controller Controller; 

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */

void drive (int Rspeed, int Lspeed, int AT){    
  RF.spin(forward,Rspeed, pct ); 
  RM.spin(forward,Rspeed, pct);
  RB.spin(forward,Rspeed, pct);
  LF.spin(forward,Lspeed, pct);
  LM.spin(forward,Lspeed, pct);
  LB.spin(forward,Lspeed, pct);
  wait(AT, msec);
}

void stop(){
  RF.stop(brake); 
  RM.stop(brake);
  RB.stop(brake);
  LF.stop(brake);
  LM.stop(brake);
  LB.stop(brake);
}

float D = 3.25;
float P = M_PI;
float G = 36/60;

void inchDrive(float target){ 
  float x = 0; 
  LF.setPosition(0, rev); 
  x = LF.position(rev)*D*P*G; 

  if (target >= 0 ){
    while (x <= target ) { 
      drive(50, 50, 10); 
      x = LF.position(rev)*D*P*G; 
    }
  }
  else if (target <0){ 
    while (x <=fabs(target)){
      drive(-50, -50, 10); 
      x = -LF.position(rev)*D*P*G;
    }
  }
  stop();
}

void gyroTurn(float target){
	float heading=0.0; //initialize a variable for heading
	float accuracy=2.0; //how accurate to make the turn in degrees
	float error=target-heading;
	float kp=5.0;
	float speed=kp*error;
	gyr.setRotation(0.0, degrees);  //reset Gyro to zero degrees
	while(fabs(error)>=accuracy){
		speed=kp*error;
		drive(speed, -speed, 10); //turn right at speed
		heading=gyr.rotation();  //measure the heading of the robot
		error=target-heading;  //calculate error
	}
	stop();  //stope the drive
}

/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  inchDrive(30);
  gyroTurn(90);
  conveyor.spin(forward, 100, pct);
  drive(75,75, 100);
  inchDrive(-24);
  outake1.spin(forward, 100, pct);
  outake2.spin(forward, 100, pct);
  inchDrive(10);
  gyroTurn(90);
  conveyor.spin(forward, 100, pct);
  inchDrive(1);

  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    int Lspeed = Controller.Axis3.position(pct);
    int Rspeed = Controller.Axis2.position(pct);
    drive(Rspeed, Lspeed, 10);

    if (Controller.ButtonL1.pressing()){ 
      outake1.spin(reverse, 100, pct); 
      outake2.spin(reverse, 100, pct); 
    }
    else if (Controller.ButtonL2.pressing()) { 
      outake1.spin(reverse, 100, pct);
      outake2.spin(forward, 100, pct);
    }
    else {
      outake1.stop();
      outake2.stop();
    }

  if (Controller.ButtonR1.pressing()){ 
    conveyor.spin(forward, 100, pct); 
  }
  else if (Controller.ButtonR2.pressing()) { 
    conveyor.spin(reverse, 100, pct);
  }
  else {
    conveyor.stop();
  }

  if (Controller.ButtonUp.pressing()){
    ScrapaparerDescorerere.set(true);
  }
  if (Controller.ButtonDown.pressing()){
    ScrapaparerDescorerere.set(false);
  }

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
