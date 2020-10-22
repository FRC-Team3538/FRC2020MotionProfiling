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
  const frc::Pose2d zero(0_ft, 0_ft, frc::Rotation2d(180_deg));
  const frc::Pose2d forward_5(5_ft, 0_ft, frc::Rotation2d(180_deg));

  std::vector<frc::Translation2d> interiorWaypoints{};

  frc::TrajectoryConfig config(5_fps, 5_fps_sq);

  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(forward_5, interiorWaypoints, zero, config);

  //wpi::outs() << "time: " << trajectory.TotalTime() << "\n";

  wpi::json states = trajectory.States();

  wpi::outs() << "states: " << states.dump() << "\n";
}
void Robot::AutonomousPeriodic() {}
void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
