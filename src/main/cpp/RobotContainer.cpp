#include "RobotContainer.h"
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

using namespace pathplanner;

RobotContainer::RobotContainer() {
  NamedCommands::registerCommand("marker1", frc2::cmd::Print("Passed marker 1"));
  NamedCommands::registerCommand("marker2", frc2::cmd::Print("Passed marker 2"));
  NamedCommands::registerCommand("print hello", frc2::cmd::Print("hello"));

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Add a button to run the example auto to SmartDashboard, this will also be in the GetAutonomousCommand method below
  testAuto = PathPlannerAuto("Example Auto").ToPtr().Unwrap();
  frc::SmartDashboard::PutData("Example Auto", exampleAuto.get());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return PathPlannerAuto("Example Auto").ToPtr();
}
