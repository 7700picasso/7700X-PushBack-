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
motor outake1 (PORT9, ratio6_1, 0);//red sprocket
motor outake2 (PORT19, ratio6_1, true);//green sprocket
motor conveyor (PORT14, ratio6_1, true);
inertial gyr (PORT1);
digital_out ScrapaparerDescorerere = digital_out(Brain.ThreeWirePort.A);
controller Controller; 

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */

void drive (int Rspeed, int Lspeed, int AT){    
  double rVolts = Rspeed * 0.12;
  double lVolts = Lspeed * 0.12;
  RF.spin(forward,rVolts, volt ); 
  RM.spin(forward,rVolts, volt);
  RB.spin(forward,rVolts, volt);
  LF.spin(forward,lVolts, volt);
  LM.spin(forward,lVolts, volt);
  LB.spin(forward,lVolts, volt);
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
float G = 36.0 / 60.0;

double revToIn(double rev){
  return rev*D*P*G;
}

double kP = 3;
double kI = 0.1;
double kD = 0;
double accuracy = 0.25;

void inchDrive(double target){ 
  //float moved = 0; 
  //RF.setPosition(0, rev); 
  double startPosition = RF.position(rev);
  double moved = revToIn(RF.position(rev) - startPosition);
  double error = target;
  double accumulatedError = 0;

  int debugCounter = 0;
  while(fabs(error) > accuracy) {
  
    double speed = kP*error + kI * accumulatedError;
    if (speed > 50){
      speed = 50;
    }
    if (speed < -50) {
      speed = -50;
    }
    drive(speed, speed, 10);
    moved = revToIn(RF.position(rev) - startPosition);
    double previousError=error;
    error = target - moved;
    if (fabs(error) < 2){
      accumulatedError= accumulatedError + error;
    }
    if ((previousError<0 && error>0) || (previousError>0 && error<0)){
      accumulatedError = 0;
    }
    if (++debugCounter % 10 == -1) {
      printf("tick: %d drive: %f moved: %0.2f error: %0.2f kP: %0.2f kI : %0.2f\n", debugCounter, target, moved, error, kP*error, kI*accumulatedError);

    }
  }
  stop();
}

void testDrive(double inches){
  double startPosition = RF.position(rev);
  long startTime = vex::timer::system();
  inchDrive(inches);
  wait(1000, msec);
  double endPosition = RF.position(rev);
  long endTime = vex::timer::system();
  printf("drive test %f: distance: %0.2f time: %lu\n", inches, revToIn(endPosition - startPosition), endTime - startTime);

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
  testDrive(40);
  testDrive(-40);
  // ..........................................................................
  /*conveyor.spin(forward, 100, pct);
  inchDrive(30);
  drive(75,75, 100);
  inchDrive(-24);
  outake1.spin(forward, 100, pct);
  outake2.spin(forward, 100, pct);
  inchDrive(10);
  gyroTurn(90);
  conveyor.spin(forward, 100, pct);
  inchDrive(1);
*/
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
  //Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
