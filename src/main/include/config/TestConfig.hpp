#pragma once
#include "ctre/Phoenix.h"
#include "lib/json.hpp"
#include "lib/ctreJsonSerde.hpp"

class TestConfig {
public:
  int a = 0;
  ctre::phoenix::CustomParamConfiguration cpc;
  ctre::phoenix::motorcontrol::can::TalonFXConfiguration fx;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TestConfig, a, cpc, fx)

