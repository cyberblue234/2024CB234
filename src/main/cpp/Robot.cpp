
#include "Teleop.h"
#include "Autonomous.h"
#include <frc/TimedRobot.h>
#include "Robot.h"

Teleop teleop;
Autonomous autonomous;

void Robot::RobotInit()
{
    swerve.ResetGyroPitch();
    swerve.ResetGyroRoll();
    swerve.ResetGyroAngle();
}

void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit()
{
    std::unique_ptr<frc2::Command> pathfindToPickup = pathplanner::AutoBuilder::followPath(pathplanner::PathPlannerPath::fromPathFile("Example Path")).Unwrap();
    // frc::SmartDashboard::PutData("Pathfind to Scoring Pos", pathfindToPickup.get());
}

void Robot::AutonomousPeriodic()
{
}

void Robot::TeleopInit()
{
    teleop.TeleopInit();
}

void Robot::TeleopPeriodic()
{
    // teleop.OperatorControls();
    frc2::CommandPtr testCmd = frc2::cmd::Print("Test");
    testCmd.Schedule();
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
