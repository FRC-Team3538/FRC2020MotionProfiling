#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <string>

#include <AHRS.h>
#include <frc/SPI.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <units/units.h>
#include <wpi/raw_ostream.h>

#include <wpi/json.h>
#include "Constants.hpp"

using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;
using namespace frc;
using namespace units;
using namespace units::angle;

class Drivebase {
public:
  Drivebase();

  void Arcade(double forward, double rotate);

  frc::Rotation2d GetGyroHeading() {
    return frc::Rotation2d(angle::degree_t(navx.GetAngle()));
  }

  void ResetOdometry() {
    auto heading = GetGyroHeading();
    odometry.ResetPosition(
        frc::Pose2d(units::meter_t(0), units::meter_t(0), heading), heading);
  }

  void UpdateOdometry() {
    odometry.Update(GetGyroHeading(),
                    units::meter_t(motorLeft1.GetSelectedSensorPosition() / Constants::ticks_per_meter),
                    units::meter_t(motorRight1.GetSelectedSensorPosition() / Constants::ticks_per_meter));
  }

  void LogState();

private:
  // Hardware setup
  enum motors { L1 = 0, L2, L3, R1, R2, R3 };

  WPI_TalonSRX motorLeft1{11};
  WPI_VictorSPX motorLeft2{13};

  WPI_TalonSRX motorRight1{12};
  WPI_VictorSPX motorRight2{14};

  AHRS navx{frc::SPI::Port::kMXP};

  DifferentialDriveKinematics kinematics{27_in};
  DifferentialDriveOdometry odometry{GetGyroHeading()};
};