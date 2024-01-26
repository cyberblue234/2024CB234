#include "RobotExt.h"
#include "Teleop.h"
#include "subsystems/Drivetrain.h"

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

	limelight3.SetPipelineID(Limelight::kAprilTag);

	//"Aimbot" PID controller config stuff, remove later
	frc::SmartDashboard::PutNumber("Rotation P", swerve.rotationP);
	frc::SmartDashboard::PutNumber("Rotation I", swerve.rotationI);
	frc::SmartDashboard::PutNumber("Rotation D", swerve.rotationD);
}

void Teleop::OperatorControls()
{
	swerve.DriveControl();

	limelight3.UpdateLimelightTracking(); //NetworkTables updating
	limelight3.UpdateLimelightDashboard(); //Updates the dashboard with the new NetworkTables data

	swerve.UpdateOdometry(); //Updates Odometry with Vision Data in mind (Make this occur on a timer later, so we can use other Pipelines as well)
	
	//"Aimbot" PID controller config stuff, remove later
	swerve.rotationP = frc::SmartDashboard::GetNumber("Rotation P", swerve.rotationP);
    swerve.rotationI = frc::SmartDashboard::GetNumber("Rotation I", swerve.rotationI);
    swerve.rotationD = frc::SmartDashboard::GetNumber("Rotation D", swerve.rotationD);
	swerve.rotcontrol.SetPID(swerve.rotationP, swerve.rotationI, swerve.rotationD);

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