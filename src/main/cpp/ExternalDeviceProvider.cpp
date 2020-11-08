#include "ExternalDeviceProvider.hpp"

flatbuffers::Offset<rj::CTREMotorStatusFrame>
GetMotorStatusFrame(flatbuffers::FlatBufferBuilder& fbb, TalonSRX& motor)
{
  Faults faults;
  motor.GetFaults(faults);

  return rj::CreateCTREMotorStatusFrame(
    fbb,
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
    0, // TODO: only call this for valid modes. motor.GetClosedLoopTarget(),
    0, // TODO: only call this for valid modes.
       // motor.GetActiveTrajectoryPosition(),
    0, // TODO: only call this for valid modes.
       // motor.GetActiveTrajectoryVelocity(),
    0, // TODO: only call this for valid modes.
       // motor.GetActiveTrajectoryArbFeedFwd(),
    faults.ToBitfield(),
    motor.HasResetOccurred(),
    motor.GetLastError(),
    static_cast<int32_t>(motor.GetControlMode()),
    motor.GetStatorCurrent(),
    motor.GetSupplyCurrent(),
    motor.IsFwdLimitSwitchClosed(),
    motor.IsRevLimitSwitchClosed());
}

flatbuffers::Offset<rj::CTREMotorStatusFrame>
GetMotorStatusFrame(flatbuffers::FlatBufferBuilder& fbb, VictorSPX& motor)
{
  Faults faults;
  motor.GetFaults(faults);

  return rj::CreateCTREMotorStatusFrame(
    fbb,
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
    0, // TODO: only call this for valid modes. motor.GetClosedLoopTarget(),
    0, // TODO: only call this for valid modes.
       // motor.GetActiveTrajectoryPosition(),
    0, // TODO: only call this for valid modes.
       // motor.GetActiveTrajectoryVelocity(),
    0, // TODO: only call this for valid modes.
       // motor.GetActiveTrajectoryArbFeedFwd(),
    faults.ToBitfield(),
    motor.HasResetOccurred(),
    motor.GetLastError(),
    static_cast<int32_t>(motor.GetControlMode()),
    0.0,
    0.0,
    0,
    0);
}

flatbuffers::Offset<rj::PDPStatusFrame>
GetPDPStatusFrame(flatbuffers::FlatBufferBuilder& fbb,
                  frc::PowerDistributionPanel& pdp)
{
  std::vector<double> currentMeasurements{
    pdp.GetCurrent(0),  pdp.GetCurrent(1),  pdp.GetCurrent(2),
    pdp.GetCurrent(3),  pdp.GetCurrent(4),  pdp.GetCurrent(5),
    pdp.GetCurrent(6),  pdp.GetCurrent(7),  pdp.GetCurrent(8),
    pdp.GetCurrent(9),  pdp.GetCurrent(10), pdp.GetCurrent(11),
    pdp.GetCurrent(12), pdp.GetCurrent(13), pdp.GetCurrent(14),
    pdp.GetCurrent(15)
  };

  return rj::CreatePDPStatusFrameDirect(
    fbb,
    0, // TODO: this is available in the beta
    pdp.GetVoltage(),
    pdp.GetTemperature(),
    &currentMeasurements,
    pdp.GetTotalCurrent(),
    pdp.GetTotalPower(),
    pdp.GetTotalEnergy());
}

flatbuffers::Offset<rj::PCMStatusFrame>
GetPCMStatusFrame(flatbuffers::FlatBufferBuilder& fbb, frc::Compressor& pcm)
{
  return rj::CreatePCMStatusFrame(fbb,
                                  0, // TODO: this is available in the beta
                                  pcm.Enabled(),
                                  pcm.GetPressureSwitchValue(),
                                  pcm.GetCompressorCurrent(),
                                  pcm.GetClosedLoopControl(),
                                  pcm.GetCompressorCurrentTooHighFault(),
                                  pcm.GetCompressorShortedFault(),
                                  pcm.GetCompressorNotConnectedFault());
}

void
ExternalDeviceProvider::BuildExternalStatusFrame(
  flatbuffers::FlatBufferBuilder& fbb, rj::StatusFrame statusFrameType, flatbuffers::Offset<void> statusFrameOffset)
{
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();

  auto statusFrameHolder =
    rj::CreateStatusFrameHolder(fbb,
                                unixTime,
                                monotonicTime,
                                statusFrameType,
                                statusFrameOffset);

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void
ExternalDeviceProvider::InitLogger()
{
  sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sockfd < 0) {
    std::cout << "could not create socket! " << strerror(errno) << std::endl;
    return;
  }

  address.sin_family = AF_INET;
  address.sin_port = htons(5801);
  if (inet_aton("10.83.32.62", &address.sin_addr) == 0) {
    std::cout << "inet_aton failed! " << strerror(errno) << std::endl;
    return;
  }
}

int sendLog(int sockfd, const flatbuffers::DetachedBuffer& buffer, const struct sockaddr_in& address) {
  if (sockfd < 0) {
    return 0;
  }

  return sendto(sockfd, buffer.data(), buffer.size(), 0, (const struct sockaddr *)&address, sizeof(address));
}

void
ExternalDeviceProvider::LogExternalDeviceStatus()
{
  if (sockfd < 0) {
    return;
  }

  fbb.Reset();
  auto pdpStatusFrameOffset = GetPDPStatusFrame(fbb, pdp);
  BuildExternalStatusFrame(fbb, rj::StatusFrame::StatusFrame_PDPStatusFrame, pdpStatusFrameOffset.Union());
  auto buffer = fbb.Release();

  if (sendLog(sockfd,
             buffer,
             address) == -1) {
    std::cout << "sendLog failed! " << strerror(errno) << std::endl;
  }

  fbb.Reset();
  auto pcmStatusFrameOffset = GetPCMStatusFrame(fbb, pcm);
  BuildExternalStatusFrame(fbb, rj::StatusFrame::StatusFrame_PCMStatusFrame, pcmStatusFrameOffset.Union());
  buffer = fbb.Release();

  if (sendLog(sockfd,
             buffer,
             address) == -1) {
    std::cout << "sendLog failed! " << strerror(errno) << std::endl;
  }

  fbb.Reset();
  auto driveLeft1StatusFrameOffset = GetMotorStatusFrame(fbb, driveLeft1);
  BuildExternalStatusFrame(fbb, rj::StatusFrame::StatusFrame_CTREMotorStatusFrame, driveLeft1StatusFrameOffset.Union());
  buffer = fbb.Release();

  if (sendLog(sockfd,
             buffer,
             address) == -1) {
    std::cout << "sendLog failed! " << strerror(errno) << std::endl;
  }

  fbb.Reset();
  auto driveLeft2StatusFrameOffset = GetMotorStatusFrame(fbb, driveLeft2);
  BuildExternalStatusFrame(fbb, rj::StatusFrame::StatusFrame_CTREMotorStatusFrame, driveLeft2StatusFrameOffset.Union());
  buffer = fbb.Release();

  if (sendLog(sockfd,
             buffer,
             address) == -1) {
    std::cout << "sendLog failed! " << strerror(errno) << std::endl;
  }

  fbb.Reset();
  auto driveRight1StatusFrameOffset = GetMotorStatusFrame(fbb, driveRight1);
  BuildExternalStatusFrame(fbb, rj::StatusFrame::StatusFrame_CTREMotorStatusFrame, driveRight1StatusFrameOffset.Union());
  buffer = fbb.Release();

  if (sendLog(sockfd,
             buffer,
             address) == -1) {
    std::cout << "sendLog failed! " << strerror(errno) << std::endl;
  }

  fbb.Reset();
  auto driveRight2StatusFrameOffset = GetMotorStatusFrame(fbb, driveRight2);
  BuildExternalStatusFrame(fbb, rj::StatusFrame::StatusFrame_CTREMotorStatusFrame, driveRight2StatusFrameOffset.Union());
  buffer = fbb.Release();

  if (sendLog(sockfd,
             buffer,
             address) == -1) {
    std::cout << "sendLog failed! " << strerror(errno) << std::endl;
  }
}