#include "Robot.h"

void Robot::RobotInit() 
{
	EnableLiveWindowInTest(true);
}

void Robot::RobotPeriodic() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic()
{
	controls.Periodic(GetPeriod());
}

void Robot::TestPeriodic()
{
	controls.Periodic(GetPeriod());
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() 
{
	controls.Periodic(GetPeriod());
	swerve.SimMode();
}

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
