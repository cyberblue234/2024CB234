#pragma once
#include <frc2/command/CommandPtr.h>
#include "subsystems/Drivetrain.h"
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"
#include "subsystems/Feeder.h"
#include "subsystems/Elevator.h"
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
		limelight3.UpdateTelemetry();
	};

	void RunTeleop();

	Drivetrain GetSwerve() { return swerve; };
	Shooter GetShooter() { return shooter; };
	Intake GetIntake() { return intake; };
	Elevator GetElevator() { return elevator; };
	Feeder GetFeeder() { return feeder; };

	void LogTeleopData();

private:
  	Drivetrain swerve;
  	Shooter shooter;
  	Intake intake;
	Elevator elevator;
	Feeder feeder;

	Controls controls;

	std::unique_ptr<frc2::Command> testAuto;

	FILE *t_output;
    frc::Timer logTimer;

	void ConfigureBindings();
};
