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

void Robot::DisabledPeriodic() {}

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
	container.RunTeleop();
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
