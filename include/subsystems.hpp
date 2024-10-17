#pragma once

#include "api.h"

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intake(3);
inline pros::MotorGroup intake2({2, -10});
// inline pros::adi::DigitalIn limit_switch('A');