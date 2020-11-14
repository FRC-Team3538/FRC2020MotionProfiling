#pragma once

#include <frc/spline/Spline.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <wpi/json.h>

#include "lib/wpilibSerde.hpp"

// class TrajConfig {
// public:
//    std::vector<Spline<5>::ControlVector> points;
//    TrajectoryConfig config;
//
//    //Trajectory get() {
//    //    return TrajectoryGenerator::generateTrajectory(points, config);
//    //}
//};
//
// NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TrajConfig, points, config)
