/*----------------------------------------------------------------------------------------*/
/*                                                                                        */      
/*    Project:          IQ2 Clawbot Project                                               */
/*    Module:           main.cpp                                                          */
/*    Author:           VEX                                                               */
/*    Created:          Fri Aug 05 2022                                                   */
/*    Description:      This is an IQ2 python Clawbot project                             */
/*                                                                                        */      
/*    Configuration:    Clawbot Template (Individual Motors + Controller)                 */
/*                      Controller                                                        */
/*                      > MOTORS:                                                         */ 
/*                      Left Motor in Port 9                                              */
/*                      Right Motor in Port 10                                            */
/*                      Claw Motor in Port 3                                              */
/*                      Arm Motor in Port 4                                               */
/*                      > SENSORS:                                                        */ 
/*                      TouchLED in Port -                                                */
/*                      Optical Sensor in Port 12                                         */
/*                      Distance Sensor in Port 8                                         */
/*                      Bumper in Port -                                                  */
/*                                                                                        */      
/*----------------------------------------------------------------------------------------*/

// Include the IQ Library
#include "vex.h"

// Allows for easier use of the VEX Library
using namespace vex;

// Brain should be defined by default
brain Brain;

// Robot configuration code.
inertial BrainInertial = inertial();
controller Controller = controller();

motor ClawMotor = motor(PORT4, false);
motor ArmMotor = motor(PORT3, true);
motor LeftDriveSmart = motor(PORT9, 1, false);
motor RightDriveSmart = motor(PORT10, 1, true);

touchled TouchLED = touchled(PORT11);
optical Optical = optical(PORT12);
distance Distance = distance(PORT8);
bumper Bumper = bumper(PORT8);

// global variables declaration.
const int MAX_SPEED_CLAW = 25; // adjust if claw grip is too tight or loose
const int MAX_SPEED_ARM = 25; // adjust if arm is moving too agressively or weakly
const int turnSensitivity = 0.25; // adjust if turns are too weak or strong
bool followingWall = false;


// calibrate drivetrain and intertial sensors
void calibrateDrivetrain() {
  wait(200, msec);
  Brain.Screen.print("Calibrating");
  Brain.Screen.newLine();
  Brain.Screen.print("Inertial");
  BrainInertial.calibrate();
  while (BrainInertial.isCalibrating()) {
    wait(25, msec);
  }

  // Clears the screen and returns the cursor to row 1, column 1.
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
}

// define a task that will handle monitoring inputs from Controller
int rc_auto_loop_function_Controller() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    // LR buttons
    // Three values, max, 0 and -max.
    int control_l1 = ((Controller.ButtonLUp.pressing() - Controller.ButtonLDown.pressing()) * MAX_SPEED_ARM );
    int control_r1; // = (Controller.ButtonRUp.pressing() - Controller.ButtonRDown.pressing()) * MAX_SPEED;
    if (Controller.ButtonRUp.pressing()) {
      control_r1 = MAX_SPEED_CLAW; // open claw (active)
    }
    if (Controller.ButtonRDown.pressing()) {
      control_r1 = -1*MAX_SPEED_CLAW; // close claw (active)
    }
    

    // calculate the drivetrain motor velocities from the controller joystick axies
    // left = AxisA
    // right = AxisD
    int drivetrainNorthSouthSpeed = -1*Controller.AxisA.position();
    int drivetrainEastWestSpeed = -1*Controller.AxisC.position();

    // threshold the variable channels so the drive does not
    // move if the joystick axis does not return exactly to 0
    const int deadband = 15;
    if(abs(drivetrainNorthSouthSpeed) < deadband) {
      drivetrainNorthSouthSpeed = 0;
    }

    if(abs(drivetrainEastWestSpeed) < deadband) {
      drivetrainEastWestSpeed = 0;
    }

    // define left and right speeds
    int drivetrainLeftSideSpeed = (drivetrainNorthSouthSpeed + turnSensitivity*drivetrainEastWestSpeed);
    int drivetrainRightSideSpeed = (drivetrainNorthSouthSpeed - turnSensitivity*drivetrainEastWestSpeed);

    // update motor velocities
    LeftDriveSmart.spin(reverse, drivetrainLeftSideSpeed, percent);
    RightDriveSmart.spin(reverse, drivetrainRightSideSpeed, percent);

    // Claw and Arm motors
    ClawMotor.spin(forward, control_l1, percent);
    ArmMotor.spin(forward, control_r1, percent);

    // wait before repeating the process
    wait(25, msec);
  }
  return 0;
}

int sensor_check() {
  while (1){
    if (Optical.isNearObject()){
      Brain.Screen.print("Optical: Near");
    } else {
      Brain.Screen.print("Optical: Far");
    }
    double distance = Distance.objectDistance(mm);
    Brain.Screen.print("Distance: %.2f", distance);

    wait(25, msec);

    Brain.Screen.clearScreen();
  }

  return 0;
}

int scan_color() {
  // debug for scan color and show in touchled
  Optical.setLight(ledState::on);
  wait(100, msec);
  color colorScanned = Optical.color();
  TouchLED.setBrightness(100);
  TouchLED.setColor(colorScanned);
  wait(100, msec);
  Optical.setLight(ledState::off);

  return 0;
}

int rc_controller_buttons_loop(){

  while (1){
    if (Controller.ButtonEUp.pressing() == true){

      Brain.playNote(4, 4, 400);  // G
      wait(50, msec);

      Brain.playNote(4, 4, 400);  // G
      wait(50, msec);

      Brain.playNote(4, 4, 400);  // G
      wait(100, msec);

      Brain.playNote(4, 1, 300);  // D
      Brain.playNote(4, 6, 150);  // B
      Brain.playNote(4, 4, 450);  // G
      wait(100, msec);

      Brain.playNote(4, 1, 300);  // D
      Brain.playNote(4, 6, 150);  // B
      Brain.playNote(4, 4, 500);  // G
    }
    if (Controller.ButtonEDown.pressing()){
    scan_color();
  }

    wait(50, msec);
  }

  return 0;
}

int main() {
  // Begin project code

  thread controllerLoop = thread(rc_auto_loop_function_Controller);
  thread sensorCheck = thread(sensor_check);
  thread controllerLoop2 = thread(rc_controller_buttons_loop);

  while(1){
    vexTaskSleep(100);
  }
    
}
