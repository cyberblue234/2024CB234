#include "Autonomous.h"

void Autonomous::RunAuto(std::string pathName) 
{
    auto path = pathplanner::PathPlannerPath::fromPathFile(pathName);

    pathplanner::AutoBuilder::followPath(path);
    // this is a test
}