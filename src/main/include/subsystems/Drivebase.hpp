#pragma once

#include <AHRS.h>

#include <ctre/Phoenix.h>
#include <adi/ADIS16470_IMU.h>

#include "ExternalDeviceProvider.hpp"

#include <frc/SPI.h>
#include <frc/controller/RamseteController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/trajectory/Trajectory.h>
#include <memory>

// #include <units/units.h>

#include "Constants.hpp"

class Drivebase
{
public:
  Drivebase(ExternalDeviceProvider &xdp): 
    imu(xdp.imu),
    motorLeft1(xdp.driveLeft1),
    motorLeft2(xdp.driveLeft2),
    motorRight1(xdp.driveRight1),
    motorRight2(xdp.driveRight2)
   {
     Configure();
   }

  void Arcade(double forward, double rotate);

  frc::Rotation2d GetGyroHeading()
  {
    return frc::Rotation2d(units::angle::degree_t(imu.GetAngle()));
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
  void Configure();

  ctre::phoenix::motorcontrol::can::TalonSRX  &motorLeft1;
  ctre::phoenix::motorcontrol::can::VictorSPX &motorLeft2;
  ctre::phoenix::motorcontrol::can::TalonSRX  &motorRight1;
  ctre::phoenix::motorcontrol::can::VictorSPX &motorRight2;

  // AHRS navx{ frc::SPI::Port::kMXP };
  frc::ADIS16470_IMU &imu;

  frc::DifferentialDriveKinematics kinematics{ 25_in };
  frc::DifferentialDriveOdometry odometry{ GetGyroHeading() };

  frc::Trajectory currentTrajectory;
  double trajectoryStartTime;
  frc::RamseteController ramsete{ 2, 0.7 };

  frc::DifferentialDriveWheelSpeeds wheelSpeeds;
};