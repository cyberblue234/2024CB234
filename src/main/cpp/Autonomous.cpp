#include "Autonomous.h"
#include <frc2/command/CommandPtr.h>

void Autonomous::RunAuto(std::string pathName) 
{
    auto path = pathplanner::PathPlannerPath::fromPathFile(pathName);

    std::unique_ptr<frc2::Command> cmd = pathplanner::AutoBuilder::followPath(path).Unwrap();
    cmd.Schedule();
}