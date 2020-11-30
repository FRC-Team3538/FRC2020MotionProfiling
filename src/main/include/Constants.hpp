#pragma once

#define _USE_MATH_DEFINES // for C++
#include <math.h>

using namespace std;

namespace Constants {

// Global Constants
const double pi = M_PI;

// Drivetrain Constants (Assuming Rectangular)
const double trackWidth = 22.25;                                      // Inches
const double wheelBase = 22.25;                                       // Inches
const double diagonal = sqrt(pow(trackWidth, 2) + pow(wheelBase, 2)); // Inches

const double ticks_per_rotation = 8192.0;
const double wheel_diameter = 0.1524;                   // meters
const double wheel_circumference = wheel_diameter * pi; // meters
const double ticks_per_meter = ticks_per_rotation / wheel_circumference;

} // namespace Constants