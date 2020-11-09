#pragma once

#include <AHRS.h>
#include <ctre/Phoenix.h>
#include <frc/SPI.h>
#include <frc/Timer.h>
#include <frc/controller/RamseteController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
// #include <units/units.h>
#include <wpi/json.h>
#include <wpi/raw_ostream.h>

#include <iostream>
#include <string>

#include "Constants.hpp"

using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;
using namespace frc;
using namespace units;
using namespace units::angle;

class Drivebase
{
public:
  Drivebase();

  void Arcade(double forward, double rotate);

  frc::Rotation2d GetGyroHeading()
  {
    return frc::Rotation2d(angle::degree_t(-navx.GetAngle()));
  }

  void ResetOdometry()
  {
    auto heading = GetGyroHeading();
    motorLeft1.SetSelectedSensorPosition(0);
    motorRight1.SetSelectedSensorPosition(0);
    odometry.ResetPosition(frc::Pose2d(0_m, 0_m, heading), heading);
  }

  void UpdateOdometry()
  {
    odometry.Update(GetGyroHeading(),
                    units::meter_t(motorLeft1.GetSelectedSensorPosition() /
                                   Constants::ticks_per_meter),
                    units::meter_t(motorRight1.GetSelectedSensorPosition() /
                                   Constants::ticks_per_meter));
  }

  void LogState();

  void SetRamseteTarget(frc::Trajectory::State state);
  void StepRamsete();
  void StopFollowing();

  void SetOpenLoop(double left, double right);

private:
  // Hardware setup
  enum motors
  {
    L1 = 0,
    L2,
    L3,
    R1,
    R2,
    R3
  };

  WPI_TalonSRX motorLeft1{ 11 };
  WPI_VictorSPX motorLeft2{ 13 };

  WPI_TalonSRX motorRight1{ 12 };
  WPI_VictorSPX motorRight2{ 14 };

  AHRS navx{ frc::SPI::Port::kMXP };

  DifferentialDriveKinematics kinematics{ 25_in };
  DifferentialDriveOdometry odometry{ GetGyroHeading() };

  frc::Trajectory currentTrajectory;
  double trajectoryStartTime;
  frc::RamseteController ramsete{ 2, 0.7 };

  frc::DifferentialDriveWheelSpeeds wheelSpeeds;
};