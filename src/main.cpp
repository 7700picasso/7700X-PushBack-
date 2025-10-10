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
motor RFM (PORT1, ratio6_1, false);
motor RMM (PORT2, ratio6_1, false);
motor RBM (PORT3, ratio6_1, false);
motor LFM (PORT4, ratio6_1, true);
motor LMM (PORT5, ratio6_1, true);
motor LBM (PORT6, ratio6_1, true);
motor FI  (PORT7, ratio6_1, true);
controller Controller;
brain Brain; 

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */

/* return type NAME (right speed, left side speed, amount of time) {

move the motor forward based on speed given by the user 
move the motor forward based on speed given by the user 
move the motor forward based on speed given by the user 
move the motor forward based on speed given by the user 
move the motor forward based on speed given by the user 
move the motor forward based on speed given by the user 
wait for certain amount of time given by the user
}*/

void drive (int Rspeed, int Lspeed, int AT){ 

  RFM.spin(forward,Rspeed, pct ); 
  RMM.spin(forward,Rspeed, pct);
  RBM.spin(forward,Rspeed, pct);
  LFM.spin(forward,Lspeed, pct);
  LMM.spin(forward,Lspeed, pct);
  LBM.spin(forward,Lspeed, pct);
  wait(AT, msec);
}


void stop(){
RFM.stop(brake); 
RMM.stop(brake);
RBM.stop(brake);
LFM.stop(brake);
LMM.stop(brake);
LBM.stop(brake);



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
  // Insert autonomous user code here.
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
