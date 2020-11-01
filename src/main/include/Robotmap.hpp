#pragma once

#include "subsystems/DS.hpp"
#include "subsystems/Drivebase.hpp"

#include "ExternalDeviceProvider.hpp"

class Robotmap
{
public:
  ExternalDeviceProvider externalDeviceProvider;

  DS ds;
  Drivebase drivebase;
};