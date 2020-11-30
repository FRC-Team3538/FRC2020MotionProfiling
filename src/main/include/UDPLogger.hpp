#pragma once

#include <functional>
#include <mutex>
#include <vector>

#include <netinet/in.h>

#include "flatbuffers/flatbuffers.h"

using namespace std;

#define FLATBUFFER_SIZE 4096

class UDPLogger
{
private:
  flatbuffers::FlatBufferBuilder fbb{ FLATBUFFER_SIZE }; // 4KB

  int sockfd;

  struct sockaddr_in address;
  std::vector<struct sockaddr_in> clients;
  std::recursive_mutex mut;
  std::string title;

public:
  void InitLogger();
  void CheckForNewClient();
  void LogWithFlatBuffer(
    std::function<void(flatbuffers::FlatBufferBuilder&)> func);
  void Log(uint8_t* data, size_t size);
  void SetTitle(std::string str);
};
