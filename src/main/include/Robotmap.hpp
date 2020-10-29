#pragma once

#include "subsystems/DS.hpp"
#include "subsystems/Drivebase.hpp"

#include "ExternalDeviceProvider.hpp"

class Robotmap
{
public:
  ExternalDeviceProvider motors;

  DS ds;
  Drivebase drivebase;
};