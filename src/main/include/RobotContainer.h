#pragma once

#include <frc2/command/CommandPtr.h>
#include "subsystems/Drivetrain.h"
#include "RobotExt.h"


class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();
  void OdometryInit() 
  {
    limelight3.UpdateLimelightTracking(); 
    swerve.ResetPose(limelight3.GetRobotPose()); 
    limelight3.UpdateLimelightDashboard();
  };
  void RunTeleop() { swerve.DriveControl(); };
  
 private:
  Drivetrain swerve;

  std::unique_ptr<frc2::Command> testAuto;

  void ConfigureBindings();
};
