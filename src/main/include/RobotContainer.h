#pragma once

#include <frc2/command/CommandPtr.h>
#include "subsystems/Drivetrain.h"
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"
#include "RobotExt.h"

class RobotContainer
{
public:
	RobotContainer();

	frc2::CommandPtr GetAutonomousCommand();
	
	void OdometryInit()
	{
		limelight3.UpdateLimelightTracking();
		swerve.ResetPose(limelight3.GetRobotPose());
		limelight3.UpdateLimelightDashboard();
	};

	void RunTeleop();

	void LogTeleopData();

private:
  	Drivetrain swerve;
  	Shooter shooter;
  	Intake intake;

	std::unique_ptr<frc2::Command> testAuto;

	FILE *t_output;
    frc::Timer logTimer;

	void ConfigureBindings();
};
