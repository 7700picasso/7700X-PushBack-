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
motor outake1 (PORT9, ratio18_1, 1);//red sprocket
motor outake2 (PORT19, ratio18_1, true);//green sprocket
motor conveyor (PORT14, ratio6_1, false);
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
float G = 36.0 / 48.0;

double revToIn(double rev){
  return rev*D*P*G;
}

double kP = 5.0;
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

void gyroTurn(float target) {

    // --- PID Constants (tune these) ---
    float kP = 0.6;
    float kI = 0.00;
    float kD = 0;

    // --- Settings ---
    float accuracy = 1.5;          // degrees tolerance
    float maxSpeed = 80;           // max turning speed (motor percent)
    int timeout = 2000;            // timeout in milliseconds

    // --- PID Variables ---
    float error = 0;
    float prevError = 0;
    float integral = 0;
    float derivative = 0;
    float output = 0;

    // --- Timer ---
    timer t;
    t.reset();

    // Optional: reset gyro
    // gyr.setRotation(0, degrees);

    while (true) {

        float heading = gyr.rotation(deg);
        error = target - heading;

        // Exit if within tolerance
        if (fabs(error) < accuracy)
            break;

        // Exit if timeout exceeded
        if (t.time(msec) > timeout)
            break;

        // --- PID Math ---
        integral += error;

        // Anti-windup (limit integral)
        if (integral > 300) integral = 300;
        if (integral < -300) integral = -300;

        derivative = error - prevError;

        output = (kP * error) + (kI * integral) + (kD * derivative);

        // Limit speed to maxSpeed
        if (output > maxSpeed) output = maxSpeed;
        if (output < -maxSpeed) output = -maxSpeed;

        // Turn robot
        drive(-output, output, 10);

        prevError = error;

        wait(10, msec);
    }

    stop();

    Brain.Screen.printAt(10, 20, "Final = %.2f", gyr.rotation(deg));
}

// void gyroTurn(float target){
// 	float heading=0.0; //initialize a variable for heading
// 	float accuracy=2.0; //how accurate to make the turn in degrees
// 	float error=target-heading;
// 	float kp=0.6;
// 	float speed=kp*error;
// 	//gyr.setRotation(0.0, degrees);  //reset Gyro to zero degrees
// 	while(fabs(error)>=accuracy){
// 		speed=kp*error;
// 		drive(-speed, speed, 10); //turn right at speed
// 		heading=gyr.rotation();  //measure the heading of the robot
// 		error=target-heading;  //calculate error
// 	}
//   Brain.Screen.printAt(10, 20, "degrees = %0.2f" , gyr.rotation(deg)); 
//   // printf("degrees=%f",gyr.rotation());
// 	stop();  //stope the drive
// }

double YOFFSET = 20; //offset for the display
//Writes a line for the diagnostics of a motor on the Brain
void MotorDisplay(double y, double curr, double temp)
{
Brain.Screen.setFillColor(transparent);
Brain.Screen.printAt(5, YOFFSET + y, "Current: %.1fA", curr);
if (curr < 1)
Brain.Screen.setFillColor(green);
else if (curr >= 1 && curr  <= 2.5)
Brain.Screen.setFillColor(yellow);
else
Brain.Screen.setFillColor(red);
Brain.Screen.drawRectangle(140, YOFFSET + y - 15, 15, 15);

Brain.Screen.setFillColor(transparent);
Brain.Screen.printAt(160, YOFFSET + y, "Temp: %.1fC", temp);
if (temp < 45)
Brain.Screen.setFillColor(green);
else if (temp <= 50 && temp  >= 45)
// TRUE and TRUE --> True
// TRUE and FALSE --> False
// FALSE and FALSE --> False
Brain.Screen.setFillColor(yellow);
else
Brain.Screen.setFillColor(red);
Brain.Screen.drawRectangle(275, YOFFSET + y - 15, 15, 15);
Brain.Screen.setFillColor(transparent);
}

///Starting the code for all the tempeture stuff

//Displays information on the brain
void Display()
{
double leftFrontCurr = LF.current(amp);
double leftFrontTemp = LF.temperature(celsius);
double leftBackCurr = LB.current(amp);
double leftBackTemp = LB.temperature(celsius);
double leftMidCurr = LM.current(amp);
double leftMidTemp = LM.temperature(celsius);

double rightFrontCurr = RF.current(amp);
double rightFrontTemp = RF.temperature(celsius);
double rightBackCurr = RB.current(amp);
double rightBackTemp = RB.temperature(celsius);
double rightMidCurr = RM.current(amp);
double rightMidTemp = RM.temperature(celsius);

double conveyorCurr = conveyor.current(amp);
double conveyorTemp = conveyor.temperature(celsius);
double outtakeCurr = outake1.current(amp);
double outtakeTemp = outake1.temperature(celsius);
///Left Section 
if (LF.installed())
{
MotorDisplay(1, leftFrontCurr, leftFrontTemp);
Brain.Screen.printAt(300, YOFFSET + 1, "LF");
}
else
Brain.Screen.printAt(5, YOFFSET + 1, "LF Problem");

if (LB.installed())
{
MotorDisplay(31, leftBackCurr, leftBackTemp);
Brain.Screen.printAt(300, YOFFSET + 31, "LB");
}
else
Brain.Screen.printAt(5, YOFFSET + 31, "LB Problem");

if (LM.installed())
{
MotorDisplay(61, leftMidCurr, leftMidTemp);
Brain.Screen.printAt(300, YOFFSET + 61, "LM");
}
else
Brain.Screen.printAt(5, YOFFSET + 61, "LM Problem");


///Right Section
if (RF.installed())
{
MotorDisplay(91, rightFrontCurr, rightFrontTemp);
Brain.Screen.printAt(300, YOFFSET + 91, "RF");
}
else
Brain.Screen.printAt(5, YOFFSET + 91, "RF Problem");

if (RB.installed())
{
MotorDisplay(121, rightBackCurr, rightBackTemp);
Brain.Screen.printAt(300, YOFFSET + 121, "RB");
}
else
Brain.Screen.printAt(5, YOFFSET + 121, "RB Problem");

if (RM.installed())
{
MotorDisplay(151, rightMidCurr, rightMidTemp);
Brain.Screen.printAt(300, YOFFSET + 151, "RM");
}
else
Brain.Screen.printAt(5, YOFFSET + 151, "RM Problem");

//other motors section
if (conveyor.installed())
{
MotorDisplay(181, conveyorCurr, conveyorTemp);
Brain.Screen.printAt(300, YOFFSET +181, "conveyor");
}
else
Brain.Screen.printAt(5, YOFFSET +181, "conveyor problem");

if (outake1.installed())
{
MotorDisplay(211, outtakeCurr, outtakeTemp);
Brain.Screen.printAt(300, YOFFSET +211, "outtake");
}
else
Brain.Screen.printAt(5, YOFFSET +211, "outtake problem");
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
  /*conveyor.spin(forward, 100, pct);
  inchDrive(30);
  gyroturn(//idk\\);
  drive(75,75, 100);
  inchDrive(-24);
  outake1.spin(forward, 100, pct);
  outake2.spin(forward, 100, pct);
  inchDrive(10);
  gyroTurn(90);
  conveyor.spin(forward, 100, pct);
  inchDrive(1);
*/
conveyor.spin(reverse,75, pct);
inchDrive(10);
gyroTurn(40);
inchDrive(20);
gyroTurn(135);
conveyor.spin(forward,75, pct);
wait(500, msec);
conveyor.stop();
conveyor.spin(reverse,75, pct);
inchDrive(32);
gyroTurn(190);
ScrapaparerDescorerere.set(true);
wait(500, msec);
inchDrive(12);
wait(1000, msec);
conveyor.spin(forward,70,pct);
inchDrive(-22);
conveyor.spin(reverse,70,pct);
outake1.spin(forward,100,pct);
outake2.spin(forward,100,pct);






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
      Display(); 

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
