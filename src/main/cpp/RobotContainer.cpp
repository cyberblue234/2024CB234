#include "RobotContainer.h"

using namespace pathplanner;

RobotContainer::RobotContainer() : swerve(GetLimelight3()), elevator(), limelight3("limelight"), limelight2("limelight-intake"),
								   pdp(1, frc::PowerDistribution::ModuleType::kRev),
								   controls(GetSwerve(), GetShooter(), GetIntake(), GetElevator(), GetFeeder(), GetLimelight3(), GetCANdle())
{
	NamedCommands::registerCommand("Shoot", GetShootCommand());
	NamedCommands::registerCommand("FirstShoot", GetFirstShootCommand());
	NamedCommands::registerCommand("Intake", GetIntakeCommand());

	autoChooser.SetDefaultOption(AutoConstants::kAutoShoot, AutoConstants::kAutoShoot);
	for (auto i = AutoConstants::kAutoArray.begin(); i != AutoConstants::kAutoArray.end(); ++i)
	{
		autoChooser.AddOption(*i, *i);
	}

	frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
	std::string auton = GetAuto();
	if (auton == AutoConstants::kAutoShoot) return GetFirstShootCommand();
	return PathPlannerAuto(auton).ToPtr();
}


frc2::CommandPtr RobotContainer::GetFirstShootCommand()
{
	return frc2::RunCommand
	(
		[this]
		{
			this->GetShooter()->ShootAtSpeaker();
		}
	).Until
	(
		[this]
		{
			return this->GetShooter()->GetAverageRPM() >= this->GetShooter()->GetSpeakerRPM() - 100;
		}
	).AndThen
	(
		frc2::RunCommand
		(
			[this]
			{
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
				this->GetShooter()->StopMotors();
				this->GetElevator()->StopMotors();
				this->GetFeeder()->StopMotor();
			}
		).ToPtr()
	);
}

frc2::CommandPtr RobotContainer::GetShootCommand()
{
	return frc2::RunCommand
	(
		[this]
		{
			this->GetShooter()->ShootAtSpeaker();
			this->GetSwerve()->AlignToSpeaker();
		}
	).Until
	(
		[this]
		{
			bool atAlignment = this->GetSwerve()->AtSetpoint();
			return this->GetShooter()->GetAverageRPM() >= this->GetShooter()->GetSpeakerRPM() - 100 && atAlignment;
		}
	).AndThen
	(
		frc2::RunCommand
		(
			[this]
			{
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
				this->GetShooter()->StopMotors();
				this->GetElevator()->StopMotors();
				this->GetFeeder()->StopMotor();
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
			this->GetIntake()->IntakeFromGround();
			this->GetFeeder()->IntakeFromGround();
		}
	).ToPtr().Until
	(
		[this]
		{
			return this->GetFeeder()->IsNoteSecured();
		}
	);
}

void RobotContainer::PlotAutonomousPath()
{
	static std::string auton = "";
	std::string newAuton = GetAuto();
	if (auton != newAuton)
	{
		if (auton != "" && auton != AutoConstants::kAutoShoot)
		{
			auto oldPathGroup = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(auton);
			char count = 48;
			for (auto path = oldPathGroup.begin(); path != oldPathGroup.end(); ++path)
			{
				swerve.GetField()->GetObject(std::string({'p', 'a', 't', 'h', count}))->SetPose(frc::Pose2d());
				count++;
			}
		}

		auton = newAuton;

		if (newAuton == AutoConstants::kAutoShoot) return;
		
		auto pathGroup = pathplanner::PathPlannerAuto::getPathGroupFromAutoFile(auton);
		char count = 48;
		for (auto path = pathGroup.begin(); path != pathGroup.end(); ++path)
		{
			swerve.GetField()->GetObject(std::string({'p', 'a', 't', 'h', count}))->SetPoses(path->get()->getPathPoses());
			count++;
		}
	}
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
				fprintf(t_output, "Time,Volts,Shooter1RPM,Shooter2RPM,FeedRPM,IntakeRPM,Elevator1RPM,Elevator2RPM,OdomX,OdomY,OdomRot\r\n");
			}
			fprintf(t_output, "%10.5f,%7.3f,%7.0f,%7.0f,%7.0f,%7.0f,%7.0f,%7.0f,%3.3f,%3.3f,%3.3f\r\n", time, volts, shooter1RPM, shooter2RPM, feedRPM, intakeRPM, elevator1RPM, elevator2RPM, odometryX, odometryY, odometryRot);
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
				fprintf(t_output, "Time,Volts,Shooter1RPM,Shooter2RPM,FeedRPM,IntakeRPM,Elevator1RPM,Elevator2RPM,OdomX,OdomY,OdomRot\r\n");
			}
			fprintf(t_output, "%10.5f,%7.3f,%7.0f,%7.0f,%7.0f,%7.0f,%7.0f,%7.0f,%3.3f,%3.3f,%3.3f\r\n", time, volts, shooter1RPM, shooter2RPM, feedRPM, intakeRPM, elevator1RPM, elevator2RPM, odometryX, odometryY, odometryRot);
		}
	}
	if (t_output != NULL && count == MAX_COUNT)
	{
		fflush(t_output);
	}
	count++;
	frc::SmartDashboard::PutNumber("LOG COUNT", count);
}