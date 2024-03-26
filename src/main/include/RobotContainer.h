#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PowerDistribution.h>

#include "subsystems/Drivetrain.h"
#include "Controls.h"

class RobotContainer
{
public:
	RobotContainer();

	Drivetrain *GetSwerve() { return &swerve; };

	void LogTeleopData();
	void LogAutoData();

private:
	Drivetrain swerve;

	frc::PowerDistribution pdp;

	Controls controls;

	FILE *t_output;
	frc::Timer logTimer;
};
