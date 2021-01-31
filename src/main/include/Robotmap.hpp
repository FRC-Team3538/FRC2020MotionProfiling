#pragma once

#include "subsystems/DS.hpp"
#include "subsystems/Drivebase.hpp"

#include "ExternalDeviceProvider.hpp"
#include "UDPLogger.hpp"

class Robotmap
{
public:
  UDPLogger logger;
  ExternalDeviceProvider externalDeviceProvider;

  DS ds;
  Drivebase drivebase{externalDeviceProvider};
};