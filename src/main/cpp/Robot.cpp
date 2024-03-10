#include "Robot.h"

void Robot::RobotInit()
{
	container.OdometryInit();
}

void Robot::RobotPeriodic()
{
	frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() 
{
	std::string newAuton = container.GetAuto();
	if (auton != newAuton)
	{
		if (auton != "" && auton != AutoConstants::kAutoShoot)
		{
			auto oldPathGroup = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(auton);
			char count = 48;
			for (auto path = oldPathGroup.begin(); path != oldPathGroup.end(); ++path)
			{
				container.GetSwerve()->GetField()->GetObject(std::string({'p', 'a', 't', 'h', count}))->SetPose(frc::Pose2d());
				count++;
			}
		}

		auton = newAuton;

		if (newAuton == AutoConstants::kAutoShoot) return;
		
		auto pathGroup = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(auton);
		char count = 48;
		for (auto path = pathGroup.begin(); path != pathGroup.end(); ++path)
		{
			container.GetSwerve()->GetField()->GetObject(std::string({'p', 'a', 't', 'h', count}))->SetPoses(path->get()->getPathPoses());
			count++;
		}
	}
}

void Robot::AutonomousInit()
{
	container.OdometryInit();

	autonomousCommand = container.GetAutonomousCommand();

	if (autonomousCommand)
	{
		autonomousCommand->Schedule();
	}
}

void Robot::AutonomousPeriodic() 
{
	container.LogAutoData();
}

void Robot::TeleopInit()
{
	container.OdometryInit();

	if (autonomousCommand)
	{
		autonomousCommand->Cancel();
	}
}

void Robot::TeleopPeriodic()
{
	container.LogTeleopData();
}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
