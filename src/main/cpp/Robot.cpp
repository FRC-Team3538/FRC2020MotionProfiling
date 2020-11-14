/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.hpp"
#include "flatbuffers/flatbuffers.h"

void
Robot::RobotInit()
{
  IO.drivebase.ResetOdometry();
  IO.externalDeviceProvider.InitLogger();
}
void
Robot::RobotPeriodic()
{
  IO.drivebase.UpdateOdometry();
  IO.externalDeviceProvider.LogExternalDeviceStatus();

  // wpi::outs() << IO.motors.GetExternalStatusFrame() << "\n";
}
void
Robot::DisabledInit()
{}
void
Robot::DisabledPeriodic()
{}
void
Robot::AutonomousInit()
{
  const frc::Pose2d zero(0_ft, 0_ft, frc::Rotation2d(0_deg));
  const frc::Pose2d forward_5(10_ft, 0_ft, frc::Rotation2d(0_deg));

  std::vector<frc::Translation2d> interiorWaypoints{
    // frc::Translation2d(10_ft, 5_ft)
  };

  frc::TrajectoryConfig config(10_fps, 6_fps_sq);

  currentTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    zero, interiorWaypoints, forward_5, config);

  auto states = currentTrajectory.States();

  autoStartTime = frc::Timer::GetFPGATimestamp();

  IO.drivebase.ResetOdometry();
}
void
Robot::AutonomousPeriodic()
{
  auto currentTime = frc::Timer::GetFPGATimestamp();
  auto state =
    currentTrajectory.Sample(units::second_t(currentTime - autoStartTime));

  wpi::json jsonState = state;

  if (units::second_t(currentTime - autoStartTime) <=
      currentTrajectory.TotalTime()) {
    // wpi::outs() << "t: " << currentTime - autoStartTime
    //             << ", state: " << jsonState.dump() << "\n";

    IO.drivebase.SetRamseteTarget(state);
    IO.drivebase.LogState();
  } else {
    IO.drivebase.StopFollowing();
  }

  // IO.drivebase.LogState();

  IO.drivebase.StepRamsete();
}
void
Robot::TeleopInit()
{}
void
Robot::TeleopPeriodic()
{
  double left = -IO.ds.Driver.GetY(frc::GenericHID::JoystickHand::kLeftHand);
  double right = -IO.ds.Driver.GetX(frc::GenericHID::JoystickHand::kRightHand);

  IO.drivebase.Arcade(left, right);
}
void
Robot::TestInit()
{}
void
Robot::TestPeriodic()
{}

#ifndef RUNNING_FRC_TESTS
int
main()
{
  return frc::StartRobot<Robot>();
}
#endif
