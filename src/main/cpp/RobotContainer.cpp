#include "RobotContainer.h"
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

using namespace pathplanner;

RobotContainer::RobotContainer() : swerve()
{
	NamedCommands::registerCommand("Shoot", std::move(shooter.GetShooterCommand()));
	NamedCommands::registerCommand("Intake", std::move(intake.GetIntakeCommand()));

	ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
	// Add a button to run the example auto to SmartDashboard, this will also be in the GetAutonomousCommand method below
	testAuto = PathPlannerAuto("Test Auto").ToPtr().Unwrap();
	frc::SmartDashboard::PutData("Test Auto", testAuto.get());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
	return PathPlannerAuto("Test Auto").ToPtr();
}

void RobotContainer::RunTeleop()
{	
	// Drivetrain Inputs
	if (gamePad.GetXButton() == true)
        swerve.SetFieldRelative(true);
    if (gamePad.GetBButton() == true)
        swerve.SetFieldRelative(false);
    
    if (gamePad.GetYButton() == true)
	    swerve.ResetGyroAngle();
    
    if (swerve.IsAlignmentOn()) 
		swerve.AlignSwerveDrive();
    else 
	{
		swerve.DriveWithInput(gamePad.GetLeftY(), gamePad.GetLeftX(), gamePad.GetRightX(), gamePad.GetLeftStickButton());
	}
	
	shooter.shootAtSpeaker = frc::SmartDashboard::GetBoolean("Shoot At Speaker?", shooter.shootAtSpeaker);

    // Intake
	if (gamePad.GetLeftTriggerAxis() > 0.2) 
		intake.IntakeFromGround();
    else if (gamePad.GetRightTriggerAxis() > 0.2)
    {
        if (shooter.shootAtSpeaker)
		{
			// todo swerve.AlignToAprilTag();
			// is the swerve at position && is the elevator at position
			//elevator.AlignShooterToSpeaker()
			shooter.ShootAtSpeaker(true);

		}
        else 
			shooter.ShootAtAmp();
    }
    else if (gamePad.GetLeftBumper()) 
		shooter.IntakeFromSource();
    else
    {
		intake.SetIntakeMotor(0.0);
        shooter.SetShooterMotors(0.0);
        feeder.SetFeedMotor(0.0);
    }

	LogTeleopData();
}

void RobotContainer::LogTeleopData()
{
#define MAX_COUNT 10000

	static long count = 0;

	double time = (double)logTimer.Get();
	double volts = pdp.GetVoltage();

	double shooter1RPM = shooter.GetShooter1RPM();
	double shooter2RPM = shooter.GetShooter2RPM();

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
				fprintf(t_output, "Time,Volts,Shooter1RPM,Shooter2RPM\r\n");
			}
			fprintf(t_output, "%10.5f,%7.3f,%7.0f,%7.0f\r\n", time, volts, shooter1RPM, shooter2RPM);
		}
	}
	if (t_output != NULL && count == MAX_COUNT)
	{
		fflush(t_output);
	}
	count++;
	frc::SmartDashboard::PutNumber("LOG COUNT", count);
}