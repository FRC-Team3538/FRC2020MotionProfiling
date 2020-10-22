/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.hpp"

void Robot::RobotInit() { IO.drivebase.ResetOdometry(); }
void Robot::RobotPeriodic() {

  IO.drivebase.UpdateOdometry();
  //IO.drivebase.LogState();
}
void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}
void Robot::AutonomousInit() {
  const frc::Pose2d zero(0_ft, 0_ft, frc::Rotation2d(0_deg));
  const frc::Pose2d forward_5(5_ft, 0_ft, frc::Rotation2d(0_deg));

  std::vector<frc::Translation2d> interiorWaypoints{};

  frc::TrajectoryConfig config(15_fps, 5_fps_sq);

  currentTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(zero, interiorWaypoints, forward_5, config);

  //wpi::outs() << "time: " << trajectory.TotalTime() << "\n";

  auto states = currentTrajectory.States();

  wpi::outs() << "states: " << states.size() << "\n";
  //for (wpi::json state: states) {
  //  wpi::outs() << "state: " << state.dump() << "\n";
  //}

  autoStartTime = frc::Timer::GetFPGATimestamp();
}
void Robot::AutonomousPeriodic() {
  auto currentTime = frc::Timer::GetFPGATimestamp();
  wpi::json state = currentTrajectory.Sample(units::second_t(currentTime - autoStartTime));
  if (units::second_t(currentTime - autoStartTime) <= currentTrajectory.TotalTime()) {
    wpi::outs() << "time: " << (currentTime - autoStartTime) << " state: " << state.dump() << "\n";
  }

}
void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
