/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drivebase.hpp"

using namespace frc;

Drivebase::Drivebase()
{
  motorLeft1.ConfigFactoryDefault();
  motorLeft2.ConfigFactoryDefault();
  motorRight1.ConfigFactoryDefault();
  motorRight2.ConfigFactoryDefault();

  motorRight1.SetInverted(true);
  motorRight2.SetInverted(true);

  motorLeft2.Follow(motorLeft1);
  motorRight2.Follow(motorRight1);

  can::TalonSRXConfiguration config;
  motorLeft1.GetAllConfigs(config);

  // do the thing
  config.primaryPID.selectedFeedbackSensor = FeedbackDevice::QuadEncoder;
  config.slot0.kF = 0.964 / 12.0;
  config.slot0.kP = 3.02 / 12.0 / 1023.0;

  motorLeft1.ConfigAllSettings(config);

  motorRight1.GetAllConfigs(config);

  // do it again
  config.primaryPID.selectedFeedbackSensor = FeedbackDevice::QuadEncoder;
  // based on observations kF is on a scale from 0 to 1, it may be from 0 to
  // 1023.
  config.slot0.kF = 0.964 / 12.0;
  // PID controller output is for sure on a scale from 0 to 1023,
  // so the kP that FRC-characterization gives us needs to be scaled
  // appropriately
  config.slot0.kP = 3.02 / 12.0 / 1023.0;

  motorRight1.ConfigAllSettings(config);

  navx.Reset();

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
  frc::Pose2d currentPose = odometry.GetPose();
  frc::ChassisSpeeds output = ramsete.Calculate(currentPose, state);
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

  motorLeft1.Set(motorcontrol::ControlMode::Velocity, left);
  motorRight1.Set(motorcontrol::ControlMode::Velocity, right);
}

void
Drivebase::StopFollowing()
{
  wheelSpeeds = DifferentialDriveWheelSpeeds{ 0_mps, 0_mps };
}

void
Drivebase::SetOpenLoop(double left, double right)
{
  // wpi::outs() << "left: " << left << " right: " << right << "\n";
  motorLeft1.Set(motorcontrol::ControlMode::PercentOutput, left);
  motorRight1.Set(motorcontrol::ControlMode::PercentOutput, right);
}
