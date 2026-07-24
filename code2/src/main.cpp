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
/*                      TouchLED in Port 1                                                */
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

motor ClawMotor = motor(PORT3, false);
motor ArmMotor = motor(PORT4, true);
motor LeftDriveSmart = motor(PORT9, 1, false);
motor RightDriveSmart = motor(PORT10, 1, true);

touchled TouchLED = touchled(PORT1);
optical Optical = optical(PORT12);
distance Distance = distance(PORT8);
bumper Bumper = bumper(PORT8);

// global variables declaration.
const int MAX_SPEED_CLAW = 25; // adjust if claw grip is too tight or loose
const int MAX_SPEED_ARM = 50; // adjust if arm is moving too agressively or weakly
const double turnSensitivity = 0.5; // adjust if turns are too weak or strong
bool isFollowingWall = false;
double targetDistance = 50.0; // default: 50.0 mm from wall detected on distance sensor
double drivetrainRightSideSpeed;
double drivetrainLeftSideSpeed;
int control_r1 = 0;
double speedMultiplier = 1;


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

int close_claw() {
  ClawMotor.spin(forward, -1*MAX_SPEED_CLAW, percent);

  wait(500, msec);

  ClawMotor.stop();
  control_r1 = 0;

  return 0;
}

int follow_wall() {
  const double kP = 0.35; // correction coefficient
  //const double baseSpeed = 40.0;        // percentage
  const double maxCorrection = 20.0;
  const double deadband = 5.0;          // mm

  if (isFollowingWall){
    // wall following algo
    double distance = Distance.objectDistance(mm);

    // // Ignore invalid or missing wall readings
    // if (distance <= 0 || distance > 500) {
    //     LeftDriveSmart.setVelocity(baseSpeed, percent);
    //     RightDriveSmart.setVelocity(baseSpeed, percent);

    //     LeftDriveSmart.spin(forward);
    //     RightMotor.spin(forward);

    //     wait(20, msec);
    //     continue;
    // }

    // Positive error means the robot is too close to the wall
    double error = targetDistance - distance;

    // Prevent constant small steering changes
    if (fabs(error) < deadband) {
        error = 0;
    }

    double correction = kP * error;

    // Limit how sharply the robot can correct
    if (correction > maxCorrection) {
        correction = maxCorrection;
    } else if (correction < -maxCorrection) {
        correction = -maxCorrection;
    }

    /*
    * Too close to right wall:
    * error is positive
    * left motor slows, right motor speeds up
    * robot steers left
    *
    * Too far from right wall:
    * error is negative
    * left motor speeds up, right motor slows
    * robot steers right
    */
    drivetrainLeftSideSpeed = drivetrainLeftSideSpeed - correction;
    drivetrainRightSideSpeed = drivetrainLeftSideSpeed + correction;

    wait(20, msec);
  }

  return 0;
}

// define a task that will handle monitoring inputs from Controller
int rc_auto_loop_function_Controller() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    // LR buttons
    // Three values, max, 0 and -max.
    int control_l1 = ((Controller.ButtonLUp.pressing() - Controller.ButtonLDown.pressing()) * MAX_SPEED_ARM );
     // = (Controller.ButtonRUp.pressing() - Controller.ButtonRDown.pressing()) * MAX_SPEED;
    if (Controller.ButtonRUp.pressing()) {
      control_r1 = MAX_SPEED_CLAW; // open claw (active)
    }
    if (Controller.ButtonRDown.pressing()) {
        control_r1 = -MAX_SPEED_CLAW;
        thread closeClaw = thread(close_claw);
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
    drivetrainLeftSideSpeed = (drivetrainNorthSouthSpeed + turnSensitivity*drivetrainEastWestSpeed)*speedMultiplier;
    drivetrainRightSideSpeed = (drivetrainNorthSouthSpeed - turnSensitivity*drivetrainEastWestSpeed)*speedMultiplier;

    follow_wall();

    // update motor velocities
    LeftDriveSmart.spin(reverse, drivetrainLeftSideSpeed, percent);
    RightDriveSmart.spin(reverse, drivetrainRightSideSpeed, percent);

    // Claw and Arm motors
    ArmMotor.spin(forward, control_l1, percent);
    if (control_r1 >= 0){
        ClawMotor.spin(forward, control_r1, percent);
    }

    // wait before repeating the process
    wait(25, msec);
  }
  return 0;
}

const char* getColorName(color detectedColor) {
    if (detectedColor == red) {
        return "Red";
    } else if (detectedColor == red_orange) {
        return "Red Orange";
    } else if (detectedColor == orange) {
        return "Orange";
    } else if (detectedColor == yellow_orange) {
        return "Yellow Orange";
    } else if (detectedColor == yellow) {
        return "Yellow";
    } else if (detectedColor == yellow_green) {
        return "Yellow Green";
    } else if (detectedColor == green) {
        return "Green";
    } else if (detectedColor == blue_green) {
        return "Blue Green";
    } else if (detectedColor == blue) {
        return "Blue";
    } else if (detectedColor == blue_violet) {
        return "Blue Violet";
    } else if (detectedColor == violet) {
        return "Violet";
    } else if (detectedColor == red_violet) {
        return "Red Violet";
    } else {
        return "Unknown";
    }
}

int scan_color() {
  // debug for scan color and show in touchled
  //Optical.setLight(ledState::on);
  //wait(100, msec);
  color colorScanned = Optical.color();
  TouchLED.setBrightness(100);
  TouchLED.setColor(colorScanned);
  //wait(100, msec);
  //Optical.setLight(ledState::off);

  return 0;
}

int sensor_check() {
  while (1){
        Brain.Screen.clearLine(1);

        // Move cursor back to row 1, column 1
        Brain.Screen.setCursor(1, 1);

        color detectedColor = Optical.color();
        // Display the latest detected colour
        Brain.Screen.print("Color: %s", getColorName(detectedColor));

        scan_color();

        if (detectedColor == blue) {
          speedMultiplier = 0.5;
        } else if (detectedColor == yellow) {
          speedMultiplier = 0.8;
        } else {
          speedMultiplier = 1;
        }

        // Refresh every 100 milliseconds
        wait(100, msec);
  }

  return 0;
}



int rc_controller_buttons_loop(){

  while (1){
    if (Controller.ButtonEUp.pressing() == true){
      // fun starwars song (idk)
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

    if (Controller.ButtonFUp.pressing()) {
      // toggle wall following
      isFollowingWall = !(isFollowingWall);
      targetDistance = Distance.objectDistance(mm);
      Brain.playNote(4,1,200);
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

  Optical.setLight(ledState::on);
  Optical.setLightPower(100, percent);

  while(1){
    vexTaskSleep(100);
  }

  return 0;
    
}
