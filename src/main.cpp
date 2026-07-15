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
/*                      Left Motor in Port 6                                              */
/*                      Right Motor in Port 1                                             */
/*                      Claw Motor in Port 4                                              */
/*                      Arm Motor in Port 10                                              */
/*                      > SENSORS:                                                        */ 
/*                      TouchLED in Port 2                                                */
/*                      Optical Sensor in Port 3                                          */
/*                      Distance Sensor in Port 7                                         */
/*                      Bumper in Port 8                                                  */
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
motor ArmMotor = motor(PORT10, true);
motor LeftDriveSmart = motor(PORT6, 1, false);
motor RightDriveSmart = motor(PORT1, 1, true);

touchled TouchLED2 = touchled(PORT2);
optical Optical3 = optical(PORT3);
distance Distance7 = distance(PORT7);
bumper Bumper8 = bumper(PORT8);

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

    const int MAX_SPEED = 50;

    // buttons
    // Three values, max, 0 and -max.
    //
    int control_l1  = (Controller.ButtonLUp.pressing() - Controller.ButtonLDown.pressing()) * MAX_SPEED;
    int control_r1  = (Controller.ButtonRUp.pressing() - Controller.ButtonRDown.pressing()) * MAX_SPEED;

    // calculate the drivetrain motor velocities from the controller joystick axies
    // left = AxisA
    // right = AxisD
    int drivetrainNorthSouthSpeed = Controller.AxisA.position();
    int drivetrainEastWestSpeed = Controller.AxisC.position();

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
    int drivetrainLeftSideSpeed = (drivetrainNorthSouthSpeed + drivetrainEastWestSpeed);
    int drivetrainRightSideSpeed = (drivetrainNorthSouthSpeed - drivetrainEastWestSpeed);

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

  while (true){
    wait(25, msec);

    // DUMMY CODE
    Brain.Screen.print("Check TouchLED...");
    Brain.Screen.newLine();
    wait(500, msec);

    Brain.Screen.print("Check Optical Sensor...");
    Brain.Screen.newLine();
    wait(500, msec);

    Brain.Screen.print("Check Distance Sensor...");
    Brain.Screen.newLine();
    Brain.Screen.print("Steering away from wall...");
    Brain.Screen.newLine();
    wait(500, msec);

    Brain.Screen.print("Check Distance Sensor...");
    Brain.Screen.newLine();
    Brain.Screen.print("Steering away from wall...");
    Brain.Screen.newLine();
    wait(500, msec);

    Brain.Screen.print("Check Bumper Sensor...");
    Brain.Screen.newLine();
    wait(500, msec);

  }

  return 0;
}

int main() {
  // Begin project code

  thread controllerLoop = thread(rc_auto_loop_function_Controller);
  thread sensorCheck = thread(sensor_check);

  while(1){
    vexTaskSleep(100);
  }
    
}
