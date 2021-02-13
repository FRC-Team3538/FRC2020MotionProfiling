#pragma once

#include <functional>
#include <mutex>
#include <vector>

#if defined(_WIN32)

#else
#include <netinet/in.h>
#endif // defined(_WIN32)

#include "flatbuffers/flatbuffers.h"

#include <ctre/Phoenix.h>
#include <frc/Compressor.h>
#include <frc/PowerDistributionPanel.h>

#include <proto/StatusFrame_generated.h>

using namespace std;

#define FLATBUFFER_SIZE 4096

class UDPLogger
{
private:
  flatbuffers::FlatBufferBuilder fbb{ FLATBUFFER_SIZE };
  uint8_t buf[FLATBUFFER_SIZE]; // 4KB
  size_t bufsize;

  int sockfd;
#if defined(_WIN32)
#else
  struct sockaddr_in address;
  std::vector<struct sockaddr_in> clients;
#endif // defined(_WIN32)
  std::recursive_mutex mut;
  std::string title;

  flatbuffers::Offset<rj::CTREMotorStatusFrame> GetStatusFrame(
    flatbuffers::FlatBufferBuilder& fbb,
    ctre::phoenix::motorcontrol::can::TalonSRX& pcm);
  flatbuffers::Offset<rj::CTREMotorStatusFrame> GetStatusFrame(
    flatbuffers::FlatBufferBuilder& fbb,
    ctre::phoenix::motorcontrol::can::VictorSPX& pcm);
  flatbuffers::Offset<rj::PDPStatusFrame> GetStatusFrame(
    flatbuffers::FlatBufferBuilder& fbb,
    frc::PowerDistributionPanel& pcm);
  flatbuffers::Offset<rj::PCMStatusFrame> GetStatusFrame(
    flatbuffers::FlatBufferBuilder& fbb,
    frc::Compressor& pcm);

  void BuildExternalDeviceFrame(
    flatbuffers::FlatBufferBuilder& fbb,
    ctre::phoenix::motorcontrol::can::TalonSRX& srx);
  void BuildExternalDeviceFrame(
    flatbuffers::FlatBufferBuilder& fbb,
    ctre::phoenix::motorcontrol::can::VictorSPX& spx);
  void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb,
                                frc::PowerDistributionPanel& pdp);
  void BuildExternalDeviceFrame(flatbuffers::FlatBufferBuilder& fbb,
                                frc::Compressor& pcm);

public:
  void InitLogger();
  void CheckForNewClient();
  void FlushLogBuffer();
  void Log(uint8_t* data, size_t size);
  void SetTitle(std::string str);

  void LogExternalDevice(ctre::phoenix::motorcontrol::can::TalonSRX& srx);
  void LogExternalDevice(ctre::phoenix::motorcontrol::can::VictorSPX& spx);
  void LogExternalDevice(frc::PowerDistributionPanel& pdp);
  void LogExternalDevice(frc::Compressor& pcm);
};
