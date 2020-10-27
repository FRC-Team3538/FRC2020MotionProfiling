#pragma once

#include "lib/Configuration.hpp"
#include "lib/ctreJsonSerde.hpp"
#include <ctre/Phoenix.h>

using namespace std;

constexpr uint32_t kLeft1 = 11;
constexpr uint32_t kLeft2 = 13;
constexpr uint32_t kRight1 = 12;
constexpr uint32_t kRight2 = 14;

class VictorSPXStatusFrame
{
public:
  VictorSPXStatusFrame(VictorSPX& motor);

  /*  identifying info */
  int firmwareVersion;
  int baseID;
  int deviceID;

  /*  status frame */
  double outputCurrent;
  double busVoltage;
  double outputPercent;
  double outputVoltage;
  double temperature;

  int selectedSensorPosition;
  int selectedSensorVelocity;

  int closedLoopError;
  double integralAccumulator;
  double errorDerivative;

  double closedLoopTarget;
  int activeTrajectoryPosition;
  int activeTrajectoryVelocity;
  double activeTrajectoryArbFeedFwd;

  Faults faults{};
  StickyFaults stickyFaults{};
  bool resetOccured;
  ErrorCode lastError;
  ControlMode controlMode;
};

class TalonSRXStatusFrame
{
public:
  TalonSRXStatusFrame(TalonSRX& motor);

  /*  identifying info */
  int firmwareVersion;
  int baseID;
  int deviceID;

  /*  status frame */
  double outputCurrent;
  double busVoltage;
  double outputPercent;
  double outputVoltage;
  double temperature;

  int selectedSensorPosition;
  int selectedSensorVelocity;

  int closedLoopError;
  double integralAccumulator;
  double errorDerivative;

  double closedLoopTarget;
  int activeTrajectoryPosition;
  int activeTrajectoryVelocity;
  double activeTrajectoryArbFeedFwd;

  Faults faults;
  StickyFaults stickyFaults;
  bool resetOccured;
  ErrorCode lastError;
  ControlMode controlMode;

  double statorCurrent;
  double supplyCurrent;
  int fwdLimitSwitchClosed;
  int revLimitSwitchClosed;
};

class MotorProvider
{
private:
  Configuration config;

  TalonSRX driveLeft1{ kLeft1 };
  TalonSRX driveRight1{ kRight1 };

  VictorSPX driveLeft2{ kLeft2 };
  VictorSPX driveRight2{ kRight2 };

  string GetMotorLog(TalonSRX& motor);
  string GetMotorLog(VictorSPX& motor);

public:
  vector<string> GetMotorLog()
  {
    vector<string> motorLogs(4);

    motorLogs.push_back(GetMotorLog(driveLeft1));
    motorLogs.push_back(GetMotorLog(driveLeft2));
    motorLogs.push_back(GetMotorLog(driveRight1));
    motorLogs.push_back(GetMotorLog(driveRight2));

    return motorLogs;
  }
};

void
to_json(wpi::json& json, const VictorSPXStatusFrame& status);

void
to_json(wpi::json& json, const TalonSRXStatusFrame& status);