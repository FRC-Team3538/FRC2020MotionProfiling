#include "MotorProvider.hpp"
//#include "lib/ctreJsonSerde.hpp"

VictorSPXStatusFrame::VictorSPXStatusFrame(VictorSPX& motor)
{
  /*  identifying info */
  firmwareVersion = motor.GetFirmwareVersion();
  baseID = motor.GetBaseID();
  deviceID = motor.GetDeviceID();

  /*  status frame */
  // outputCurrent = motor.GetOutputCurrent();
  busVoltage = motor.GetBusVoltage();
  outputPercent = motor.GetMotorOutputPercent();
  outputVoltage = motor.GetMotorOutputVoltage();
  temperature = motor.GetTemperature();

  selectedSensorPosition = motor.GetSelectedSensorPosition();
  selectedSensorVelocity = motor.GetSelectedSensorVelocity();

  closedLoopError = motor.GetClosedLoopError();
  integralAccumulator = motor.GetIntegralAccumulator();
  errorDerivative = motor.GetErrorDerivative();

  closedLoopTarget = motor.GetClosedLoopTarget();
  activeTrajectoryPosition = motor.GetActiveTrajectoryPosition();
  activeTrajectoryVelocity = motor.GetActiveTrajectoryVelocity();
  activeTrajectoryArbFeedFwd = motor.GetActiveTrajectoryArbFeedFwd();

  motor.GetFaults(faults);
  motor.GetStickyFaults(stickyFaults);
  resetOccured = motor.HasResetOccurred();
  lastError = motor.GetLastError();
  controlMode = motor.GetControlMode();
}

TalonSRXStatusFrame::TalonSRXStatusFrame(TalonSRX& motor)
{
  /*  identifying info */
  firmwareVersion = motor.GetFirmwareVersion();
  baseID = motor.GetBaseID();
  deviceID = motor.GetDeviceID();

  /*  status frame */
  outputCurrent = motor.GetOutputCurrent();
  busVoltage = motor.GetBusVoltage();
  outputPercent = motor.GetMotorOutputPercent();
  outputVoltage = motor.GetMotorOutputVoltage();
  temperature = motor.GetTemperature();

  selectedSensorPosition = motor.GetSelectedSensorPosition();
  selectedSensorVelocity = motor.GetSelectedSensorVelocity();

  closedLoopError = motor.GetClosedLoopError();
  integralAccumulator = motor.GetIntegralAccumulator();
  errorDerivative = motor.GetErrorDerivative();

  closedLoopTarget = motor.GetClosedLoopTarget();
  activeTrajectoryPosition = motor.GetActiveTrajectoryPosition();
  activeTrajectoryVelocity = motor.GetActiveTrajectoryVelocity();
  activeTrajectoryArbFeedFwd = motor.GetActiveTrajectoryArbFeedFwd();

  motor.GetFaults(faults);
  motor.GetStickyFaults(stickyFaults);
  resetOccured = motor.HasResetOccurred();
  lastError = motor.GetLastError();
  controlMode = motor.GetControlMode();

  statorCurrent = motor.GetStatorCurrent();
  supplyCurrent = motor.GetSupplyCurrent();
  fwdLimitSwitchClosed = motor.IsFwdLimitSwitchClosed();
  revLimitSwitchClosed = motor.IsRevLimitSwitchClosed();
}

string
MotorProvider::GetMotorLog(TalonSRX& motor)
{
  auto statusFrame = TalonSRXStatusFrame(motor);
  wpi::json j;
  to_json(j, statusFrame);
  return j.dump();
}

string
MotorProvider::GetMotorLog(VictorSPX& motor)
{
  auto statusFrame = VictorSPXStatusFrame(motor);
  wpi::json j;
  to_json(j, statusFrame);
  return j.dump();
}

void
to_json(wpi::json& json, const VictorSPXStatusFrame& status)
{
  json = wpi::json({
    { "firmwareVersion", status.firmwareVersion },
    { "baseID", status.baseID },
    { "deviceID", status.deviceID },
    { "outputCurrent", status.outputCurrent },
    { "busVoltage", status.busVoltage },
    { "outputPercent", status.outputPercent },
    { "outputVoltage", status.outputVoltage },
    { "temperature", status.temperature },
    { "selectedSensorPosition", status.selectedSensorPosition },
    { "selectedSensorVelocity", status.selectedSensorVelocity },
    { "closedLoopError", status.closedLoopError },
    { "integralAccumulator", status.integralAccumulator },
    { "errorDerivative", status.errorDerivative },
    { "closedLoopTarget", status.closedLoopTarget },
    { "activeTrajectoryPosition", status.activeTrajectoryPosition },
    { "activeTrajectoryVelocity", status.activeTrajectoryVelocity },
    { "activeTrajectoryArbFeedFwd", status.activeTrajectoryArbFeedFwd },
    //{ "faults", status.faults },
    //{ "stickyFaults", status.stickyFaults },
    { "resetOccured", status.resetOccured },
    { "lastError", status.lastError },
    { "controlMode", status.controlMode },
  });
}

void
to_json(wpi::json& json, const TalonSRXStatusFrame& status)
{
  json = wpi::json({
    { "firmwareVersion", status.firmwareVersion },
    { "baseID", status.baseID },
    { "deviceID", status.deviceID },
    { "outputCurrent", status.outputCurrent },
    { "busVoltage", status.busVoltage },
    { "outputPercent", status.outputPercent },
    { "outputVoltage", status.outputVoltage },
    { "temperature", status.temperature },
    { "selectedSensorPosition", status.selectedSensorPosition },
    { "selectedSensorVelocity", status.selectedSensorVelocity },
    { "closedLoopError", status.closedLoopError },
    { "integralAccumulator", status.integralAccumulator },
    { "errorDerivative", status.errorDerivative },
    { "closedLoopTarget", status.closedLoopTarget },
    { "activeTrajectoryPosition", status.activeTrajectoryPosition },
    { "activeTrajectoryVelocity", status.activeTrajectoryVelocity },
    { "activeTrajectoryArbFeedFwd", status.activeTrajectoryArbFeedFwd },
    //{ "faults", status.faults },
    //{ "stickyFaults", status.stickyFaults },
    { "resetOccured", status.resetOccured },
    { "lastError", status.lastError },
    { "controlMode", status.controlMode },
    { "statorCurrent", status.statorCurrent },
    { "supplyCurrent", status.supplyCurrent },
    { "fwdLimitSwitchClosed", status.fwdLimitSwitchClosed },
    { "revLimitSwitchClosed", status.revLimitSwitchClosed },
  });
}
