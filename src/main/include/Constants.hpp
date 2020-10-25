#pragma once

#include <cmath>

using namespace std;

namespace Constants
{

// Global Constants
const double pi = M_PI;

// Drivetrain Constants (Assuming Rectangular)
const double trackWidth = 22.25; // Inches
const double wheelBase = 22.25;  // Inches
const double diagonal = sqrt(pow(trackWidth, 2) + pow(wheelBase, 2)); // Inches


const double ticks_per_meter =  8555.107579;

} // namespace Constants