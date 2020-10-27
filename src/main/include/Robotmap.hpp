#pragma once

#include "subsystems/DS.hpp"
#include "subsystems/Drivebase.hpp"

#include "MotorProvider.hpp"

class Robotmap
{
public:
  MotorProvider motors;

  DS ds;
  Drivebase drivebase;
};