#include "autons.hpp"
#include <cmath>
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "helpers.hpp"
#include "main.h"
#include "subsystems.hpp"
/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////


// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

///
// Constants
///

void default_constants() {
  chassis.pid_heading_constants_set(11, 0, 20);
  chassis.pid_drive_constants_set(20, 0, 100);
  chassis.pid_turn_constants_set(3, 0.05, 20, 15);
  chassis.pid_swing_constants_set(6, 0, 65);

  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.slew_drive_constants_set(7_in, 80);
}

//highstakes autonomous routines

void highstakes_blue_left() {
  mogo.set(false);
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  mogo.set(true);

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  intake.move(-127);
  chassis.pid_drive_set(36_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  intake.move(0);
}

void highstakes_blue_right() {
  mogo.set(false);
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  mogo.set(true);

  chassis.pid_wait();
  mogo.set(true);

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  intake.move(-127);
  chassis.pid_drive_set(36_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-68_deg, TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_drive_set(12_in, DRIVE_SPEED);
  chassis.pid_wait();
  intake.move(0); 
}

void highstakes_red_left() {
  mogo.set(false);
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  mogo.set(true);

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  intake.move(-127);
  chassis.pid_drive_set(36_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(68_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(12_in, DRIVE_SPEED);
  chassis.pid_wait();
  intake.move(0);
}

void highstakes_red_right() {
  mogo.set(false);
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  mogo.set(true);

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  intake.move(-127);
  chassis.pid_drive_set(36_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  intake.move(0);
}

//skills auton

void skills() {
  mogo.set(false);
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  mogo.set(true);

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  intake.move(-127);
  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-18_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(34_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-6_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(36_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  
  chassis.pid_turn_set(-80_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(84_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  mogo.set(false);
}

//tuner autons
void drive_48(){
  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}
void drive_96(){
  chassis.pid_drive_set(96_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}
void driveBack_48(){
  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-48_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}
void driveBack_96(){
  chassis.pid_drive_set(96_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-96_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}
void turn_90(){
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}
void turn_180(){
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
}
void turn_360(){
  chassis.pid_turn_set(360_deg, TURN_SPEED);
  chassis.pid_wait();
}
void turnBack(){
  chassis.pid_turn_set(360_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}