#include "ExternalDeviceProvider.hpp"

rj::CTREMotorStatusFrame
GetMotorStatusFrame(TalonSRX& motor)
{
  Faults faults;
  motor.GetFaults(faults);

  return rj::CTREMotorStatusFrame{ motor.GetFirmwareVersion(),
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
                                   motor.IsRevLimitSwitchClosed() };
}

rj::CTREMotorStatusFrame
GetMotorStatusFrame(VictorSPX& motor)
{
  Faults faults;
  motor.GetFaults(faults);

  return rj::CTREMotorStatusFrame{ motor.GetFirmwareVersion(),
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
                                   0 };
}

rj::PDPStatusFrame
GetPDPStatusFrame(frc::PowerDistributionPanel& pdp)
{
  return rj::PDPStatusFrame{ 0, // TODO: this is available in the beta
                             pdp.GetVoltage(),
                             pdp.GetTemperature(),
                             pdp.GetCurrent(0),
                             pdp.GetCurrent(1),
                             pdp.GetCurrent(2),
                             pdp.GetCurrent(3),
                             pdp.GetCurrent(4),
                             pdp.GetCurrent(5),
                             pdp.GetCurrent(6),
                             pdp.GetCurrent(7),
                             pdp.GetCurrent(8),
                             pdp.GetCurrent(9),
                             pdp.GetCurrent(10),
                             pdp.GetCurrent(11),
                             pdp.GetCurrent(12),
                             pdp.GetCurrent(13),
                             pdp.GetCurrent(14),
                             pdp.GetCurrent(15),
                             pdp.GetTotalCurrent(),
                             pdp.GetTotalPower(),
                             pdp.GetTotalEnergy() };
}

rj::PCMStatusFrame
GetPCMStatusFrame(frc::Compressor& pcm)
{
  return rj::PCMStatusFrame{ 0, // TODO: this is available in the beta
                             pcm.Enabled(),
                             pcm.GetPressureSwitchValue(),
                             pcm.GetCompressorCurrent(),
                             pcm.GetClosedLoopControl(),
                             pcm.GetCompressorCurrentTooHighFault(),
                             pcm.GetCompressorShortedFault(),
                             pcm.GetCompressorNotConnectedFault() };
}

flatbuffers::Offset<rj::StatusFrameCollection>
ExternalDeviceProvider::GetExternalStatusFrame(
  flatbuffers::FlatBufferBuilder& fbb)
{
  auto driveLeft1StatusFrame = GetMotorStatusFrame(driveLeft1);
  auto driveLeft2StatusFrame = GetMotorStatusFrame(driveLeft2);
  auto driveRight1StatusFrame = GetMotorStatusFrame(driveRight1);
  auto driveRight2StatusFrame = GetMotorStatusFrame(driveRight2);
  auto pdpStatusFrame = GetPDPStatusFrame(pdp);
  auto pcmStatusFrame = GetPCMStatusFrame(pcm);

  return rj::CreateStatusFrameCollection(fbb,
                                         &driveLeft1StatusFrame,
                                         &driveLeft2StatusFrame,
                                         &driveRight1StatusFrame,
                                         &driveRight2StatusFrame,
                                         &pdpStatusFrame,
                                         &pcmStatusFrame);
}
