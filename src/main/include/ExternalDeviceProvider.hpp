#pragma once

#include "flatbuffers/flatbuffers.h"
#include "lib/Configuration.hpp"
#include "lib/ctreJsonSerde.hpp"
#include "proto/StatusFrame_generated.h"
#include <ctre/Phoenix.h>
#include <frc/Compressor.h>
#include <frc/PowerDistributionPanel.h>

using namespace std;

constexpr uint32_t kLeft1 = 11;
constexpr uint32_t kLeft2 = 13;
constexpr uint32_t kRight1 = 12;
constexpr uint32_t kRight2 = 14;

class ExternalDeviceProvider
{
private:
  Configuration config;

  TalonSRX driveLeft1{ kLeft1 };
  TalonSRX driveRight1{ kRight1 };

  VictorSPX driveLeft2{ kLeft2 };
  VictorSPX driveRight2{ kRight2 };

  frc::PowerDistributionPanel pdp{};
  frc::Compressor pcm{};

public:
  flatbuffers::Offset<rj::StatusFrameCollection> GetExternalStatusFrame(
    flatbuffers::FlatBufferBuilder& fbb);
};