#pragma once

#include "lib/Configuration.hpp"
#include "lib/ctreJsonSerde.hpp"
#include "proto/StatusFrame_generated.h"
#include "flatbuffers/flatbuffers.h"
#include <ctre/Phoenix.h>

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

  rj::CTREMotorStatusFrame GetMotorStatusFrame(TalonSRX& motor) {
    Faults faults;
    motor.GetFaults(faults);

    return rj::CTREMotorStatusFrame{
      motor.GetFirmwareVersion(),
      motor.GetBaseID(),
      motor.GetDeviceID(),
      motor.GetOutputCurrent(),
      motor.GetBusVoltage(),
      motor.GetMotorOutputPercent(),
      motor.GetMotorOutputVoltage(),
      motor.GetTemperature(),
      motor.GetSelectedSensorPosition(),
      motor.GetSelectedSensorVelocity(),
      motor.GetClosedLoopError(),
      motor.GetIntegralAccumulator(),
      motor.GetErrorDerivative(),
      motor.GetClosedLoopTarget(),
      motor.GetActiveTrajectoryPosition(),
      motor.GetActiveTrajectoryVelocity(),
      motor.GetActiveTrajectoryArbFeedFwd(),
      faults.ToBitfield(),
      motor.HasResetOccurred(),
      motor.GetLastError(),
      static_cast<int32_t>(motor.GetControlMode()),
      motor.GetStatorCurrent(),
      motor.GetSupplyCurrent(),
      motor.IsFwdLimitSwitchClosed(),
      motor.IsRevLimitSwitchClosed()
    };
  }

  rj::CTREMotorStatusFrame GetMotorStatusFrame(VictorSPX& motor) {
    Faults faults;
    motor.GetFaults(faults);

    return rj::CTREMotorStatusFrame{
      motor.GetFirmwareVersion(),
      motor.GetBaseID(),
      motor.GetDeviceID(),
      0.0,
      motor.GetBusVoltage(),
      motor.GetMotorOutputPercent(),
      motor.GetMotorOutputVoltage(),
      motor.GetTemperature(),
      motor.GetSelectedSensorPosition(),
      motor.GetSelectedSensorVelocity(),
      motor.GetClosedLoopError(),
      motor.GetIntegralAccumulator(),
      motor.GetErrorDerivative(),
      motor.GetClosedLoopTarget(),
      motor.GetActiveTrajectoryPosition(),
      motor.GetActiveTrajectoryVelocity(),
      motor.GetActiveTrajectoryArbFeedFwd(),
      faults.ToBitfield(),
      motor.HasResetOccurred(),
      motor.GetLastError(),
      static_cast<int32_t>(motor.GetControlMode()),
      0.0,
      0.0,
      0,
      0
    };
  }

public:
  flatbuffers::Offset<rj::StatusFrameCollection> GetExternalStatusFrame(flatbuffers::FlatBufferBuilder &fbb)
  {
    auto driveLeft1StatusFrame = GetMotorStatusFrame(driveLeft1);
    auto driveLeft2StatusFrame = GetMotorStatusFrame(driveLeft2);
    auto driveRight1StatusFrame = GetMotorStatusFrame(driveRight1);
    auto driveRight2StatusFrame = GetMotorStatusFrame(driveRight2);
    
    return rj::CreateStatusFrameCollection(
      fbb,
      &driveLeft1StatusFrame,
      &driveLeft2StatusFrame,
      &driveRight1StatusFrame,
      &driveRight2StatusFrame
    );
  }
};
