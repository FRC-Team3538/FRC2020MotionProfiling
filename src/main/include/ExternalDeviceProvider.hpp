#pragma once

#include "UDPLogger.hpp"
#include "flatbuffers/flatbuffers.h"

#include "proto/StatusFrame_generated.h"
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

  void PopulateLogBuffer(UDPLogger& fbb);
};
