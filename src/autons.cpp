#include "autons.hpp"
#include <cmath>
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "helpers.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"
/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////


// These are out of 127
const int DRIVE_SPEED = 100;
const int TURN_SPEED = 80;
const int SWING_SPEED = 90;
const int SKILLS_DRIVE_SPEED = 70;
const int SKILLS_TURN_SPEED = 70;
///
// Constants
///

void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there  
  
}

//highstakes autonomous routines

void highstakes_blue_left() {
  mogo.set(false);
  chassis.pid_drive_set(-36_in, 40, true);
  chassis.pid_wait_until(-30_in);
  mogo.set(true);
  chassis.pid_speed_max_set(DRIVE_SPEED);
  chassis.pid_wait();
  mogo.set(true);

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  intake.move(-127);
  pros::delay(2000);
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();

  pros::delay(2000);

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(2000);
  intake.move(0);
}

void highstakes_blue_right() {
  mogo.set(false);
  chassis.pid_drive_set(-36_in, 40, true);
  chassis.pid_wait_until(-30_in);
  mogo.set(true);
  chassis.pid_speed_max_set(DRIVE_SPEED);
  chassis.pid_wait();
  mogo.set(true);

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  intake.move(-127);
  pros::delay(2000);
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  pros::delay(2000);

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(48_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(2000);
  intake.move(0);
}

void highstakes_red_left() {
  mogo.set(false);
  chassis.pid_drive_set(-36_in, 40, true);
  chassis.pid_wait_until(-30_in);
  mogo.set(true);
  chassis.pid_speed_max_set(DRIVE_SPEED);
  chassis.pid_wait();
  mogo.set(true);

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  intake.move(-127);
  pros::delay(2000);
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  pros::delay(2000);

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(48_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(2000);
  intake.move(0);
}

void highstakes_red_right() {
  mogo.set(false);
  chassis.pid_drive_set(-36_in, 40, true);
  chassis.pid_wait_until(-30_in);
  mogo.set(true);
  chassis.pid_speed_max_set(DRIVE_SPEED);
  chassis.pid_wait();
  mogo.set(true);

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  intake.move(-127);
  pros::delay(2000);
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();

  pros::delay(2000);

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  chassis.pid_wait();
   pros::delay(2000);
  intake.move(0);
}

//skills auton
void skills() {
//positions itself for the 1st mogo grab
  chassis.pid_drive_set(15_in, SKILLS_DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-90_deg, SKILLS_DRIVE_SPEED);
  chassis.pid_wait();
//grabs 1st mogo
  mogo.set(false);
  chassis.pid_drive_set(-30_in, 40, true);
  chassis.pid_wait_until(-24_in);
  mogo.set(true);
  chassis.pid_speed_max_set(SKILLS_DRIVE_SPEED);
  chassis.pid_wait();
  mogo.set(true);
//offsets distance traveled by mogo grab 
  chassis.pid_drive_set(6_in, SKILLS_DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, SKILLS_TURN_SPEED);
  chassis.pid_wait();
//gets 2nd ring and feeds 1st ring (preload)
  intake.move(-127);
  pros::delay(2000);
  chassis.pid_drive_set(24_in,SKILLS_DRIVE_SPEED, true);
  chassis.pid_wait(); 

  pros::delay(2000);
//gets 3rd ring
  chassis.pid_drive_set(12_in, SKILLS_DRIVE_SPEED, true);
  chassis.pid_wait();

  pros::delay(2000);
//position itself for second mogo deposit
  chassis.pid_drive_set(-10_in, SKILLS_DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, SKILLS_TURN_SPEED);
  chassis.pid_wait();
//deposits 1st mogo in corner
  chassis.pid_drive_set(-12_in, SKILLS_DRIVE_SPEED, true);
  chassis.pid_wait();
  mogo.set(false);
  pros::delay(2000);
//positions itself for 2nd mogo grab
  chassis.pid_drive_set(12_in, SKILLS_DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, SKILLS_TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-48_in, SKILLS_DRIVE_SPEED, true);
  chassis.pid_wait();
//grabs 2nd mogo
  chassis.pid_drive_set(-30_in, 40, true);
  chassis.pid_wait_until(-24_in);
  mogo.set(true);
  chassis.pid_speed_max_set(SKILLS_DRIVE_SPEED);
  chassis.pid_wait();
  mogo.set(true);
//offsets distance traveled for 2nd mogo grab
  chassis.pid_drive_set(6_in, SKILLS_DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-90_deg, SKILLS_TURN_SPEED);
  chassis.pid_wait();
//grabs 1st ring
  chassis.pid_drive_set(24_in, SKILLS_DRIVE_SPEED, true);
  chassis.pid_wait();

  pros::delay(2000);
//grabs 2nd ring
  chassis.pid_drive_set(12_in, SKILLS_DRIVE_SPEED, true);
  chassis.pid_wait();

  pros::delay(2000);
//positions itself for 2nd mogo deposit
  chassis.pid_drive_set(-12_in, SKILLS_DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, SKILLS_TURN_SPEED);
  chassis.pid_wait();
//deposits 2nd mogo in corner
  chassis.pid_drive_set(-12_in, SKILLS_DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(2000);
  mogo.set(false);
}