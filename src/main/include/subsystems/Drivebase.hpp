#pragma once

#include <AHRS.h>

#include <ctre/Phoenix.h>

#include <adi/ADIS16470_IMU.h>
#include <frc/SPI.h>
#include <frc/controller/RamseteController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/trajectory/Trajectory.h>
#include <lib/CTREUtil.hpp>
#include <lib/DrivebaseConfig.hpp>

// #include <units/units.h>

#include "Constants.hpp"
#include "lib/Loggable.hpp"
#include <UDPLogger.hpp>

class Drivebase : public rj::Loggable
{
public:
  Drivebase(rj::DrivebaseConfig& config)
    : config(config)
    , motorLeft1(
        ctre::phoenix::motorcontrol::can::WPI_TalonSRX{ config.driveLeft1.id })
    , motorLeft2(
        ctre::phoenix::motorcontrol::can::WPI_VictorSPX{ config.driveLeft2.id })
    , motorRight1(
        ctre::phoenix::motorcontrol::can::WPI_TalonSRX{ config.driveRight1.id })
    , motorRight2(ctre::phoenix::motorcontrol::can::WPI_VictorSPX{
        config.driveRight2.id })
    , imu{ config.imu.yaw_axis, config.imu.port, config.imu.cal_time }
  {
    rj::ConfigureWPI_TalonSRX(
      motorLeft1, config.driveLeft1, config.talonConfig);
    rj::ConfigureWPI_VictorSPX(
      motorLeft2, config.driveLeft2, config.victorConfig);
    rj::ConfigureWPI_TalonSRX(
      motorRight1, config.driveRight1, config.talonConfig);
    rj::ConfigureWPI_VictorSPX(
      motorRight2, config.driveRight2, config.victorConfig);
    Configure();
  };

  void Configure();

  void Log(UDPLogger& logger)
  {
    logger.LogExternalDevice(motorLeft1);
    logger.LogExternalDevice(motorLeft2);
    logger.LogExternalDevice(motorRight1);
    logger.LogExternalDevice(motorRight2);
  }

  void Arcade(double forward, double rotate);

  frc::Rotation2d GetGyroHeading()
  {
    return frc::Rotation2d(units::angle::degree_t(imu.GetAngle()));
  }

  void ResetOdometry()
  {
    imu.Reset();
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
  rj::DrivebaseConfig& config;

  ctre::phoenix::motorcontrol::can::WPI_TalonSRX motorLeft1;
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX motorLeft2;
  ctre::phoenix::motorcontrol::can::WPI_TalonSRX motorRight1;
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX motorRight2;

  // AHRS &navx;
  frc::ADIS16470_IMU imu;

  frc::DifferentialDriveKinematics kinematics{ 25_in };
  frc::DifferentialDriveOdometry odometry{ GetGyroHeading() };

  frc::Trajectory currentTrajectory;
  double trajectoryStartTime;
  frc::RamseteController ramsete{ 2, 0.7 };

  frc::DifferentialDriveWheelSpeeds wheelSpeeds;
};