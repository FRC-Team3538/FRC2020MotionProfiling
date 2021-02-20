#pragma once

#include "subsystems/DS.hpp"
#include "subsystems/Drivebase.hpp"

#include "UDPLogger.hpp"
#include "lib/Configuration.hpp"

class Robotmap
{
private:
  Configuration config;
  std::string systemMapFile{ "DrivebaseConfig.json" };
  rj::DrivebaseConfig driveConfig{ config.Get<rj::DrivebaseConfig>(
    systemMapFile) };

public:
  UDPLogger logger;

  DS ds;
  Drivebase drivebase{ driveConfig };

  std::vector<std::shared_ptr<rj::Loggable>> loggables{
    std::shared_ptr<rj::Loggable>(&drivebase)
  };
};