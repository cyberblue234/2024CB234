#include "RobotExt.h"
#include "Teleop.h"
#include "Drivetrain.h"

Teleop::Teleop()
{
	logTimer.Start();
}

void Teleop::TeleopInit()
{
	swerve.ResetCancoders();
	swerve.ResetGyroAngle();
	swerve.ResetGyroPitch();
	swerve.ResetGyroRoll();
	swerve.ResetDriveEncoders();
	swerve.SetDriveOpenLoopRamp(0.0);
}

void Teleop::OperatorControls()
{
	swerve.DriveControl();
	swerve.UpdateOdometry();

	LogTeleopData();
}

void Teleop::LogTeleopData()
{
#define MAX_COUNT 6750

	static long count = 0;

	double time = (double)logTimer.Get();
	double volts = pdp.GetVoltage();
	double distance = swerve.GetDriveDistance();

	double fld_curr = swerve.GetFLDriveCurrent();
	double fls_curr = swerve.GetFLSwerveCurrent();
	double frd_curr = swerve.GetFRDriveCurrent();
	double frs_curr = swerve.GetFRSwerveCurrent();
	double bld_curr = swerve.GetBLDriveCurrent();
	double bls_curr = swerve.GetBLSwerveCurrent();
	double brd_curr = swerve.GetBRDriveCurrent();
	double brs_curr = swerve.GetBRSwerveCurrent();

	if (count == 0)
	{
		t_output = fopen("/cb234/teleopdata.csv", "w");
		logTimer.Reset();
	}
	if (count < MAX_COUNT)
	{
		if (t_output != NULL)
		{
			if (count == 0)
			{
				fprintf(t_output, "Time,Volts,Distance,FL_D,FL_S,FR_D,FR_S,BL_D,BL_S,BR_D,BR_S\r\n");
			}
			fprintf(t_output, "%10.5f,%7.3f,%6.1f,%6.1f,%6.1f,%6.1f,%6.1f,%6.1f,%6.1f,%6.1f,%6.1f\r\n", time, volts, distance, fld_curr, fls_curr, frd_curr, frs_curr, bld_curr, bls_curr, brd_curr, brs_curr);
		}
	}
	if (t_output != NULL && count == MAX_COUNT)
	{
		fflush(t_output);
	}
	count++;
	frc::SmartDashboard::PutNumber("LOG COUNT", count);
}