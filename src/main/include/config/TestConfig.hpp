#pragma once
#include "ctre/Phoenix.h"
#include "lib/ctreJsonSerde.hpp"
#include "lib/json.hpp"

class TestConfig
{
public:
  int a = 0;
  ctre::phoenix::CustomParamConfiguration cpc;
  ctre::phoenix::motorcontrol::can::TalonFXConfiguration fx;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TestConfig, a, cpc, fx)
