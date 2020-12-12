#pragma once

#include <AHRS.h>

#include <ctre/Phoenix.h>

#include <frc/SPI.h>
#include <frc/controller/RamseteController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/trajectory/Trajectory.h>

// #include <units/units.h>

#include "Constants.hpp"

class Drivebase
{
public:
  Drivebase();

  void Arcade(double forward, double rotate);

  frc::Rotation2d GetGyroHeading()
  {
    return frc::Rotation2d(units::angle::degree_t(-navx.GetAngle()));
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
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX motorLeft1{ 11 };
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX motorLeft2{ 13 };
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX motorRight1{ 12 };
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX motorRight2{ 14 };

  AHRS navx{ frc::SPI::Port::kMXP };

  frc::DifferentialDriveKinematics kinematics{ 25_in };
  frc::DifferentialDriveOdometry odometry{ GetGyroHeading() };

  frc::Trajectory currentTrajectory;
  double trajectoryStartTime;
  frc::RamseteController ramsete{ 2, 0.7 };

  frc::DifferentialDriveWheelSpeeds wheelSpeeds;
};