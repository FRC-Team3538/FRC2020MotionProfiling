#pragma once

#include <string>
#include <ctre/Phoenix.h>
#include <iostream>

using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;
using namespace frc;
class Drivebase
{
public:
  Drivebase();
  void Arcade(double forward, double rotate);

private:
  // Hardware setup
  enum motors
  {
    L1 = 0,
    L2,
    L3,
    R1,
    R2,
    R3
  };

  WPI_TalonSRX motorLeft1{motors::L1};
  WPI_VictorSPX motorLeft2{motors::L2};

  WPI_TalonSRX motorRight1{motors::R1};
  WPI_VictorSPX motorRight2{motors::R2};
};