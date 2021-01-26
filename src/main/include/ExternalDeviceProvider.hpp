#pragma once


#include "flatbuffers/flatbuffers.h"
#include "lib/Configuration.hpp"
#include "lib/ctreJsonSerde.hpp"
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


using namespace std;

constexpr uint32_t kLeft1 = 11;
constexpr uint32_t kLeft2 = 13;
constexpr uint32_t kRight1 = 12;
constexpr uint32_t kRight2 = 14;

class ExternalDeviceProvider
{
private:
  Configuration config;

  flatbuffers::FlatBufferBuilder fbb;

public:
  TalonSRX driveLeft1{ kLeft1 };
  TalonSRX driveRight1{ kRight1 };

  VictorSPX driveLeft2{ kLeft2 };
  VictorSPX driveRight2{ kRight2 };

  frc::PowerDistributionPanel pdp{};
  frc::Compressor pcm{};

  frc::ADIS16470_IMU imu;

  void PopulateLogBuffer(UDPLogger& fbb);
};
