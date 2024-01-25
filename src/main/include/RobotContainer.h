#pragma once

#include <frc2/command/CommandPtr.h>

#include "Drivetrain.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  Drivetrain swerve;

  std::unique_ptr<frc2::Command> testAuto;

  void ConfigureBindings();
};
