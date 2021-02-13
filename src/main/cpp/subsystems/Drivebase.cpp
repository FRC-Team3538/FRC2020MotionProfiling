/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drivebase.hpp"

// #include <frc/Timer.h>
// #include <frc/geometry/Pose2d.h>
// #include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
// #include <units/units.h>
#include <wpi/json.h>
#include <wpi/raw_ostream.h>
//
// #include <iostream>
// #include <string>
//

// using namespace frc;

void
Drivebase::Configure()
{
  motorLeft2.Follow(motorLeft1);
  motorRight2.Follow(motorRight1);

  imu.Reset();
  imu.Calibrate();

  motorLeft1.SetSelectedSensorPosition(0);
  motorRight1.SetSelectedSensorPosition(0);

  ResetOdometry();
}

void
Drivebase::Arcade(double forward, double turn)
{
  // Constrain input to +/- 1.0
  if (std::abs(forward) > 1.0) {
    forward /= std::abs(forward);
  }
  if (std::abs(turn) > 1.0) {
    turn /= std::abs(turn);
  }

  motorLeft1.Set(forward - turn);
  motorRight1.Set(forward + turn);
}

void
Drivebase::LogState()
{

  auto pose = odometry.GetPose();
  wpi::json j;
  frc::to_json(j, pose);
  auto timestamp = frc::Timer::GetFPGATimestamp();
  wpi::outs() << "[" << timestamp << "]: lv:"
              << motorLeft1.GetSelectedSensorVelocity() * 10 /
                   Constants::ticks_per_meter
              << ", rv:"
              << motorRight1.GetSelectedSensorVelocity() * 10 /
                   Constants::ticks_per_meter
              << " => " << j.dump() << "\n";
}

void
Drivebase::SetRamseteTarget(frc::Trajectory::State state)
{
  auto currentPose = odometry.GetPose();
  auto output = ramsete.Calculate(currentPose, state);
  wheelSpeeds = kinematics.ToWheelSpeeds(output);
}

void
Drivebase::StepRamsete()
{
  double left =
    wheelSpeeds.left.to<double>() * Constants::ticks_per_meter / 10.0;
  double right =
    wheelSpeeds.right.to<double>() * Constants::ticks_per_meter / 10.0;

  wpi::outs() << "\tleft: " << wheelSpeeds.left.to<double>()
              << " right: " << wheelSpeeds.right.to<double>() << "\n";

  motorLeft1.Set(motorcontrol::ControlMode::Velocity,
                 left,
                 motorcontrol::DemandType::DemandType_ArbitraryFeedForward,
                 config.kSLinear / 12.0);
  motorRight1.Set(motorcontrol::ControlMode::Velocity,
                  right,
                  motorcontrol::DemandType::DemandType_ArbitraryFeedForward,
                  config.kSLinear / 12.0);
}

void
Drivebase::StopFollowing()
{
  wheelSpeeds = frc::DifferentialDriveWheelSpeeds{ 0_mps, 0_mps };
}

void
Drivebase::SetOpenLoop(double left, double right)
{
  // wpi::outs() << "left: " << left << " right: " << right << "\n";
  motorLeft1.Set(motorcontrol::ControlMode::PercentOutput,
                 left,
                 motorcontrol::DemandType::DemandType_ArbitraryFeedForward,
                 config.kSLinear / 12.0);
  motorRight1.Set(motorcontrol::ControlMode::PercentOutput,
                  right,
                  motorcontrol::DemandType::DemandType_ArbitraryFeedForward,
                  config.kSLinear / 12.0);
}
