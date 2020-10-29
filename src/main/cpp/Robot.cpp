/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.hpp"
#include "flatbuffers/flatbuffers.h"
#include <iostream>

void
Robot::RobotInit()
{
  IO.drivebase.ResetOdometry();
  //wpi::outs() << sizeof(rj::StatusFrameCollection) << "\n";
  flatbuffers::FlatBufferBuilder fbb{};
  IO.motors.GetExternalStatusFrame(fbb);
  std::cout << fbb.GetSize() << std::endl;
}
void
Robot::RobotPeriodic()
{

  IO.drivebase.UpdateOdometry();
  // IO.drivebase.LogState();
  //wpi::outs() << IO.motors.GetExternalStatusFrame() << "\n";

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
  const frc::Pose2d forward_5(5_ft, 0_ft, frc::Rotation2d(0_deg));

  std::vector<frc::Translation2d> interiorWaypoints{};

  frc::TrajectoryConfig config(5_fps, 5_fps_sq);

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
    wpi::outs() << "t: " << currentTime - autoStartTime
                << ", state: " << jsonState.dump() << "\n";

    IO.drivebase.SetRamseteTarget(state);
  } else {
    IO.drivebase.StopFollowing();
  }

  IO.drivebase.LogState();

  IO.drivebase.StepRamsete();
}
void
Robot::TeleopInit()
{}
void
Robot::TeleopPeriodic()
{
  double left = -IO.ds.Driver.GetY(frc::GenericHID::JoystickHand::kLeftHand);
  double right = -IO.ds.Driver.GetY(frc::GenericHID::JoystickHand::kRightHand);

  IO.drivebase.SetOpenLoop(left, right);
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
