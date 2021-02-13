#include <iostream>

#if defined(_WIN32)

#else
#include <arpa/inet.h>
#include <netinet/ip.h>
#include <sys/socket.h>
#endif // defined(_WIN32)

#include "frc/Timer.h"

#include "UDPLogger.hpp"
#include <assert.h>

#include <iostream>

#if defined(_WIN32)

void
UDPLogger::InitLogger()
{}

void
UDPLogger::CheckForNewClient()
{}

void
UDPLogger::FlushLogBuffer()
{}

#else

void
UDPLogger::InitLogger()
{

  sockfd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, IPPROTO_UDP);
  if (sockfd < 0) {
    std::cout << "socket() failed! " << strerror(errno) << std::endl;
    return;
  }

  address.sin_family = AF_INET;
  address.sin_port = htons(3538);

  if (inet_aton("0.0.0.0", &address.sin_addr) == 0) {
    std::cout << "inet_aton() failed! " << strerror(errno) << std::endl;
    return;
  }

  if (bind(sockfd, (const struct sockaddr*)&address, sizeof(address)) != 0) {
    std::cout << "bind() failed! " << strerror(errno) << std::endl;
    return;
  }
}

int
sendLog(int sockfd,
        const uint8_t* data,
        size_t size,
        const struct sockaddr_in& address)
{
  if (sockfd < 0) {
    return 0;
  }

  return sendto(
    sockfd, data, size, 0, (const struct sockaddr*)&address, sizeof(address));
}

// Need to lock at this level so we don't cause iterator invalidation on
// `clients`
void
UDPLogger::FlushLogBuffer()
{
  mut.lock();
  for (struct sockaddr_in addr : clients) {
    if (sendLog(sockfd, buf, bufsize, addr) == -1) {
      std::cout << "sendLog failed! " << strerror(errno) << std::endl;
    }
  }
  bufsize = 0;
  mut.unlock();
}

#define RECV_BUF_SIZE 3

void
UDPLogger::CheckForNewClient()
{
  struct sockaddr_in client;
  socklen_t client_len = sizeof(struct sockaddr_in);
  char buf[RECV_BUF_SIZE];
  buf[2] = 0x00;
  ssize_t res_len = recvfrom(sockfd,
                             (void*)buf,
                             RECV_BUF_SIZE,
                             0,
                             (struct sockaddr*)&client,
                             &client_len);

  if (res_len == 2 && strcmp(buf, "Hi") == 0) {
    mut.lock();
    clients.push_back(client);

    fbb.Reset();
    auto greeting = rj::CreateInitializeStatusFrameDirect(fbb, title.c_str());
    auto wrapper =
      rj::CreateStatusFrameHolder(fbb,
                                  frc::GetTime(),
                                  frc::Timer::GetFPGATimestamp(),
                                  rj::StatusFrame_InitializeStatusFrame,
                                  greeting.Union());
    fbb.FinishSizePrefixed(wrapper);
    auto buffer = fbb.Release();

    sendLog(sockfd, buffer.data(), buffer.size(), client);

    mut.unlock();
  }
}

#endif // defined(_WIN32)

void
UDPLogger::Log(uint8_t* data, size_t size)
{
  // shoutouts to memory safety
  if (bufsize + size > FLATBUFFER_SIZE) {
    FlushLogBuffer();
  }
  assert(bufsize + size < FLATBUFFER_SIZE);
  memcpy(buf + bufsize, data, size);
  bufsize += size;
}

void
UDPLogger::SetTitle(std::string str)
{
  title = str;
}

#include "time.h"
#include <cmath>
#include <cstdlib>

flatbuffers::Offset<rj::CTREMotorStatusFrame>
UDPLogger::GetStatusFrame(flatbuffers::FlatBufferBuilder& fbb, TalonSRX& motor)
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
UDPLogger::GetStatusFrame(flatbuffers::FlatBufferBuilder& fbb, VictorSPX& motor)
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
UDPLogger::GetStatusFrame(flatbuffers::FlatBufferBuilder& fbb,
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

  return rj::CreatePDPStatusFrameDirect(fbb,
                                        pdp.GetModule(),
                                        pdp.GetVoltage(),
                                        pdp.GetTemperature(),
                                        &currentMeasurements,
                                        pdp.GetTotalCurrent(),
                                        pdp.GetTotalPower(),
                                        pdp.GetTotalEnergy());
}

flatbuffers::Offset<rj::PCMStatusFrame>
UDPLogger::GetStatusFrame(flatbuffers::FlatBufferBuilder& fbb,
                          frc::Compressor& pcm)
{
  return rj::CreatePCMStatusFrame(fbb,
                                  pcm.GetModule(),
                                  pcm.Enabled(),
                                  pcm.GetPressureSwitchValue(),
                                  pcm.GetCompressorCurrent(),
                                  pcm.GetClosedLoopControl(),
                                  pcm.GetCompressorCurrentTooHighFault(),
                                  pcm.GetCompressorShortedFault(),
                                  pcm.GetCompressorNotConnectedFault());
}

void
UDPLogger::BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb,
                                    TalonSRX& motor)
{
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, motor);

  auto statusFrameHolder = rj::CreateStatusFrameHolder(
    fbb,
    unixTime,
    monotonicTime,
    rj::StatusFrame::StatusFrame_CTREMotorStatusFrame,
    statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void
UDPLogger::BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb,
                                    VictorSPX& motor)
{
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, motor);

  auto statusFrameHolder = rj::CreateStatusFrameHolder(
    fbb,
    unixTime,
    monotonicTime,
    rj::StatusFrame::StatusFrame_CTREMotorStatusFrame,
    statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void
UDPLogger::BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb,
                                    frc::PowerDistributionPanel& pdp)
{
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, pdp);

  auto statusFrameHolder =
    rj::CreateStatusFrameHolder(fbb,
                                unixTime,
                                monotonicTime,
                                rj::StatusFrame::StatusFrame_PDPStatusFrame,
                                statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void
UDPLogger::BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb,
                                    frc::Compressor& pcm)
{
  auto unixTime = frc::GetTime();
  auto monotonicTime = frc::Timer::GetFPGATimestamp();
  auto statusFrameOffset = GetStatusFrame(fbb, pcm);

  auto statusFrameHolder =
    rj::CreateStatusFrameHolder(fbb,
                                unixTime,
                                monotonicTime,
                                rj::StatusFrame::StatusFrame_PCMStatusFrame,
                                statusFrameOffset.Union());

  rj::FinishSizePrefixedStatusFrameHolderBuffer(fbb, statusFrameHolder);
}

void
UDPLogger::LogExternalDevice(ctre::phoenix::motorcontrol::can::TalonSRX& srx)
{
  fbb.Reset();
  BuildExternalDeviceFrame(fbb, srx);
  auto buffer = fbb.Release();
  Log(buffer.data(), buffer.size());
}
void
UDPLogger::LogExternalDevice(ctre::phoenix::motorcontrol::can::VictorSPX& srx)
{
  fbb.Reset();
  BuildExternalDeviceFrame(fbb, srx);
  auto buffer = fbb.Release();
  Log(buffer.data(), buffer.size());
}
void
UDPLogger::LogExternalDevice(frc::PowerDistributionPanel& srx)
{
  fbb.Reset();
  BuildExternalDeviceFrame(fbb, srx);
  auto buffer = fbb.Release();
  Log(buffer.data(), buffer.size());
}
void
UDPLogger::LogExternalDevice(frc::Compressor& srx)
{
  fbb.Reset();
  BuildExternalDeviceFrame(fbb, srx);
  auto buffer = fbb.Release();
  Log(buffer.data(), buffer.size());
}