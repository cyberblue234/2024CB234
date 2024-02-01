#include "RobotContainer.h"
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

using namespace pathplanner;

RobotContainer::RobotContainer() : swerve() {}

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