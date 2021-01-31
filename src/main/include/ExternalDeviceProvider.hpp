#pragma once

#include "UDPLogger.hpp"
#include "flatbuffers/flatbuffers.h"

#include "proto/StatusFrame_generated.h"
#include <ctre/Phoenix.h>
#include <frc/Compressor.h>
#include <frc/PowerDistributionPanel.h>
#include <iostream>

#include "lib/Configuration.hpp"
#include "lib/SystemMap.hpp"

#include <adi/ADIS16470_IMU.h>


using namespace std;

class ExternalDeviceProvider
{
private:
  flatbuffers::FlatBufferBuilder fbb;

  Configuration config;
  std::string systemMapFile{"SystemMap.json"};
  rj::SystemMap map{config.Get<rj::SystemMap>(systemMapFile)};

public:
  ExternalDeviceProvider()
  {
    wpi::json j;
    rj::to_json(j, map);
    std::cout << "MAP!: " << j.dump() << std::endl;
    driveLeft1.SetInverted(map.driveLeft1.invertType);
    driveRight1.SetInverted(map.driveRight1.invertType);
    driveLeft2.SetInverted(map.driveLeft2.invertType);
    driveRight2.SetInverted(map.driveRight2.invertType);
    driveLeft1.SetNeutralMode(map.driveLeft1.neutralMode);
    driveRight1.SetNeutralMode(map.driveRight1.neutralMode);
    driveLeft2.SetNeutralMode(map.driveLeft2.neutralMode);
    driveRight2.SetNeutralMode(map.driveRight2.neutralMode);
  }

  WPI_TalonSRX driveLeft1  {map.driveLeft1.id};
  WPI_TalonSRX driveRight1 {map.driveRight1.id};
  WPI_VictorSPX driveLeft2 {map.driveLeft2.id};
  WPI_VictorSPX driveRight2{map.driveRight2.id};

  frc::PowerDistributionPanel pdp;
  frc::Compressor pcm;

  // AHRS navx{ frc::SPI::Port::kMXP };

  frc::ADIS16470_IMU imu;

  void PopulateLogBuffer(UDPLogger& fbb);
};
