#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Shooter.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

  void RunTeleop();

 private:
  Drivetrain swerve;
  Shooter shooter;
};
