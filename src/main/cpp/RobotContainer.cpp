#include "RobotContainer.h"

using namespace pathplanner;

RobotContainer::RobotContainer() : swerve(),
								   pdp(1, frc::PowerDistribution::ModuleType::kRev),
								   controls(GetSwerve())
{
}

void RobotContainer::LogTeleopData()
{
	#define MAX_COUNT 10000

	static long count = 0;

	double time = (double)logTimer.Get();
	double volts = pdp.GetVoltage();

	frc::Pose2d robotPose = swerve.GetPose();
	double odometryX = (double) robotPose.X();
	double odometryY = (double) robotPose.Y();
	double odometryRot = (double) robotPose.Rotation().Degrees();


	if (count == 0)
	{
		t_output = fopen("/cb234/teleopdata.csv", "w");
		logTimer.Start();
		logTimer.Reset();
	}
	if (count < MAX_COUNT)
	{
		if (t_output != NULL)
		{
			if (count == 0)
			{
				fprintf(t_output, "Time,Volts,OdomX,OdomY,OdomRot\r\n");
			}
			fprintf(t_output, "%10.5f,%7.3f,%3.3f,%3.3f,%3.3f\r\n", time, volts, odometryX, odometryY, odometryRot);
		}
	}
	if (t_output != NULL && count == MAX_COUNT)
	{
		fflush(t_output);
	}
	count++;
	frc::SmartDashboard::PutNumber("LOG COUNT", count);
}

void RobotContainer::LogAutoData()
{
	#define MAX_COUNT 10000

	static long count = 0;

	double time = (double)logTimer.Get();
	double volts = pdp.GetVoltage();

	frc::Pose2d robotPose = swerve.GetPose();
	double odometryX = (double) robotPose.X();
	double odometryY = (double) robotPose.Y();
	double odometryRot = (double) robotPose.Rotation().Degrees();


	if (count == 0)
	{
		t_output = fopen("/cb234/teleopdata.csv", "w");
		logTimer.Start();
		logTimer.Reset();
	}
	if (count < MAX_COUNT)
	{
		if (t_output != NULL)
		{
			if (count == 0)
			{
				fprintf(t_output, "Time,Volts,OdomX,OdomY,OdomRot\r\n");
			}
			fprintf(t_output, "%10.5f,%7.3f,%3.3f,%3.3f,%3.3f\r\n", time, volts, odometryX, odometryY, odometryRot);
		}
	}
	if (t_output != NULL && count == MAX_COUNT)
	{
		fflush(t_output);
	}
	count++;
	frc::SmartDashboard::PutNumber("LOG COUNT", count);
}