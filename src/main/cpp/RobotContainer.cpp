#include "RobotContainer.h"

using namespace pathplanner;

RobotContainer::RobotContainer() : swerve(GetLimelight3()), elevator(GetLimelight3()), limelight3("limelight"), limelight2("limelight-intake"),
								   pdp(1, frc::PowerDistribution::ModuleType::kRev),
								   controls(GetSwerve(), GetShooter(), GetIntake(), GetElevator(), GetFeeder(), GetLimelight3(), GetOrchestra())
{
	NamedCommands::registerCommand("Shoot", GetShootCommand());
	NamedCommands::registerCommand("Intake", GetIntakeCommand());
	NamedCommands::registerCommand("ShooterMotorsOn", GetShooterMotorsOnCommand());

	autoChooser.SetDefaultOption(AutoConstants::kAutoShoot, AutoConstants::kAutoShoot);
	for (auto i = AutoConstants::kAutoArray.begin(); i != AutoConstants::kAutoArray.end(); ++i)
	{
		autoChooser.AddOption(*i, *i);
	}

	frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

	orchestra.AddInstrument(*swerve.GetFrontLeftModule()->GetDriveMotor());
	orchestra.AddInstrument(*swerve.GetFrontLeftModule()->GetSwerveMotor());
	orchestra.AddInstrument(*swerve.GetFrontRightModule()->GetDriveMotor());
	orchestra.AddInstrument(*swerve.GetFrontRightModule()->GetSwerveMotor());
	orchestra.AddInstrument(*swerve.GetBackLeftModule()->GetDriveMotor());
	orchestra.AddInstrument(*swerve.GetBackLeftModule()->GetSwerveMotor());
	orchestra.AddInstrument(*swerve.GetBackRightModule()->GetDriveMotor());
	orchestra.AddInstrument(*swerve.GetBackRightModule()->GetSwerveMotor());
	orchestra.AddInstrument(*elevator.GetElevator1Motor());
	orchestra.AddInstrument(*elevator.GetElevator2Motor());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
	std::string auton = GetAuto();
	if (auton == AutoConstants::kAutoShoot) return GetShootCommand();
	return PathPlannerAuto(auton).ToPtr();
}

frc2::CommandPtr RobotContainer::GetShootCommand()
{
	return frc2::RunCommand
	(
		[this]
		{
			this->GetSwerve()->AlignToSpeaker();
			this->GetElevator()->ElevatorControl(this->GetElevator()->CalculateSpeakerAngle());
		}
	).Until
	(
		[this]
		{
			bool atAlignment = this->GetSwerve()->AtSetpoint() && this->GetElevator()->AtSetpoint();
			return this->GetShooter()->GetShooter1RPM() >= this->GetShooter()->GetSpeakerRPM() - 100 && atAlignment;
		}
	).AndThen
	(
		frc2::RunCommand
		(
			[this]
			{
				this->GetShooter()->ShootAtSpeaker();
				this->GetFeeder()->ShootAtSpeaker();
			}
		).Until
		(
			[this]
			{
				return this->GetFeeder()->GetTopSensorInput() == false;
			}
		)
	).AndThen
	(
		frc2::InstantCommand
		(
			[this]
			{
				this->GetShooter()->SetShooterMotors(0.0);
				this->GetElevator()->SetElevatorMotors(0.0);
			}
		).ToPtr()
	);
}

frc2::CommandPtr RobotContainer::GetIntakeCommand()
{
	return frc2::RunCommand
	(
		[this]
		{
			this->GetElevator()->ElevatorControl(this->GetElevator()->GetIntakeAngle());
		}
	).ToPtr().Until
	(
		[this]
		{
			return this->GetElevator()->AtSetpoint();
		}
	).AndThen
	(
		frc2::RunCommand
		(
			[this]
			{
				this->GetIntake()->IntakeFromGround();
				this->GetFeeder()->IntakeFromGround();
			}
		).Until
		(
			[this]
			{
				return this->GetFeeder()->IsNoteSecured() == true;
			}
		)
	);
}

frc2::CommandPtr RobotContainer::GetShooterMotorsOnCommand()
{
	return frc2::RunCommand
	(
		[this]
		{
			this->GetShooter()->ShootAtSpeaker();
		}
	).ToPtr();
}

void RobotContainer::LogTeleopData()
{
	#define MAX_COUNT 10000

	static long count = 0;

	double time = (double)logTimer.Get();
	double volts = pdp.GetVoltage();

	double shooter1RPM = shooter.GetShooter1RPM();
	double shooter2RPM = shooter.GetShooter2RPM();
	double feedRPM = feeder.GetFeedMotorRPM();
	double intakeRPM = intake.GetIntakeMotorRPM();
	double elevator1RPM = elevator.GetElevator1MotorRPM();
	double elevator2RPM = elevator.GetElevator2MotorRPM();
	double shooterAngle = elevator.GetShooterAngle();
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
				fprintf(t_output, "Time,Volts,Shooter1RPM,Shooter2RPM,FeedRPM,IntakeRPM,Elevator1RPM,Elevator2RPM,ShooterAngle,OdomX,OdomY,OdomRot\r\n");
			}
			fprintf(t_output, "%10.5f,%7.3f,%7.0f,%7.0f,%7.0f,%7.0f,%7.0f,%7.0f,%3.3f,%3.3f,%3.3f,%3.3f\r\n", time, volts, shooter1RPM, shooter2RPM, feedRPM, intakeRPM, elevator1RPM, elevator2RPM, shooterAngle, odometryX, odometryY, odometryRot);
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

	double shooter1RPM = shooter.GetShooter1RPM();
	double shooter2RPM = shooter.GetShooter2RPM();
	double feedRPM = feeder.GetFeedMotorRPM();
	double intakeRPM = intake.GetIntakeMotorRPM();
	double elevator1RPM = elevator.GetElevator1MotorRPM();
	double elevator2RPM = elevator.GetElevator2MotorRPM();
	double shooterAngle = elevator.GetShooterAngle();
	frc::Pose2d robotPose = swerve.GetPose();
	double odometryX = (double) robotPose.X();
	double odometryY = (double) robotPose.Y();
	double odometryRot = (double) robotPose.Rotation().Degrees();

	if (count == 0)
	{
		t_output = fopen("/cb234/autodata.csv", "w");
		logTimer.Start();
		logTimer.Reset();
	}
	if (count < MAX_COUNT)
	{
		if (t_output != NULL)
		{
			if (count == 0)
			{
				fprintf(t_output, "Time,Volts,Shooter1RPM,Shooter2RPM,FeedRPM,IntakeRPM,Elevator1RPM,Elevator2RPM,ShooterAngle,OdomX,OdomY,OdomRot\r\n");
			}
			fprintf(t_output, "%10.5f,%7.3f,%7.0f,%7.0f,%7.0f,%7.0f,%7.0f,%7.0f,%3.3f,%3.3f,%3.3f,%3.3f\r\n", time, volts, shooter1RPM, shooter2RPM, feedRPM, intakeRPM, elevator1RPM, elevator2RPM, shooterAngle, odometryX, odometryY, odometryRot);
		}
	}
	if (t_output != NULL && count == MAX_COUNT)
	{
		fflush(t_output);
	}
	count++;
	frc::SmartDashboard::PutNumber("LOG COUNT", count);
}