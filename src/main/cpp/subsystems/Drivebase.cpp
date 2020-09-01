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
}

void Drivebase::Arcade(double forward, double turn)
{
    // Constrain input to +/- 1.0
    if (std::abs(forward) > 1.0)
    {
        forward /= std::abs(forward);
    }
    if (std::abs(turn) > 1.0)
    {
        turn /= std::abs(turn);
    }

    motorLeft1.Set(forward - turn);
    motorLeft2.Set(forward - turn);
    motorRight1.Set(forward + turn);
    motorRight2.Set(forward + turn);
}
    