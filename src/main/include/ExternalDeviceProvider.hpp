#pragma once


#include "flatbuffers/flatbuffers.h"

#include "proto/StatusFrame_generated.h"
#include "rev/CANSparkMax.h"
#include "rev/ColorSensorV3.h"
#include "UDPLogger.hpp"
#include <adi/ADIS16470_IMU.h>
#include <AHRS.h>
#include <ctre/Phoenix.h>
#include <frc/Compressor.h>
#include <frc/PowerDistributionPanel.h>
#include <iostream>

#include "lib/Configuration.hpp"

#include <adi/ADIS16470_IMU.h>

using namespace std;

class ExternalDeviceProvider
{
private:
  flatbuffers::FlatBufferBuilder fbb;

public:
  ExternalDeviceProvider() {}

  // frc::PowerDistributionPanel pdp;
  // frc::Compressor pcm;

  frc::ADIS16470_IMU imu;

  void PopulateLogBuffer(UDPLogger& fbb);
};
