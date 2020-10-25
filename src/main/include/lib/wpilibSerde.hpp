#pragma once

#include <frc/spline/Spline.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <units/units.h>
#include <wpi/json.h>

// namespace frc {
// NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Spline<5>::ControlVector, x, y)
//
// void to_json(nlohmann::json &j, const TrajectoryConfig &config) {
//  j = nlohmann::json{{"maxVelocity", config.MaxVelocity()},
//                     {"maxAcceleration", config.MaxAcceleration()},
//                     {"startVelocity", config.StartVelocity()},
//                     {"endVelocity", config.EndVelocity()},
//                     //{"reversed", config.IsReversed()}
//                     };
//}
//
// void from_json(const nlohmann::json &j, TrajectoryConfig &config) {
//  config.setMaxVelocity(j["maxVelocity"].get<units::meters_per_second_t>())
//}
//} // namespace frc
