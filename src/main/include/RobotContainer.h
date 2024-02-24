#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/PowerDistribution.h>
#include "subsystems/Drivetrain.h"
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"
#include "subsystems/Feeder.h"
#include "subsystems/Elevator.h"
#include "subsystems/Limelight.h"
#include "Controls.h"

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

	Drivetrain *GetSwerve() { return &swerve; };
	Shooter *GetShooter() { return &shooter; };
	Intake *GetIntake() { return &intake; };
	Elevator *GetElevator() { return &elevator; };
	Feeder *GetFeeder() { return &feeder; };
	Limelight *GetLimelight3() { return &limelight3; };
	Limelight *GetLimelight2() { return &limelight2; };

	void LogTeleopData();

private:
  	Drivetrain swerve;
  	Shooter shooter;
  	Intake intake;
	Elevator elevator;
	Feeder feeder;
	Limelight limelight3;
	Limelight limelight2;

	frc::PowerDistribution pdp;

	Controls controls;

	std::unique_ptr<frc2::Command> testAuto;

	FILE *t_output;
    frc::Timer logTimer;

	void ConfigureBindings();
};
