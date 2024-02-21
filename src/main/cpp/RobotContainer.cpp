#include "RobotContainer.h"
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

using namespace pathplanner;

RobotContainer::RobotContainer() : swerve()
{
	NamedCommands::registerCommand("Shoot", std::move(shooter.GetShooterCommand()));
	NamedCommands::registerCommand("Intake", std::move(intake.GetIntakeCommand()));

	ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
	// Add a button to run the example auto to SmartDashboard, this will also be in the GetAutonomousCommand method below
	testAuto = PathPlannerAuto("Test Auto").ToPtr().Unwrap();
	frc::SmartDashboard::PutData("Test Auto", testAuto.get());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
	return PathPlannerAuto("Test Auto").ToPtr();
}

void RobotContainer::RunTeleop()
{
    swerve.DriveControl();
    shooter.ShooterControl();
    intake.IntakeControl();
}