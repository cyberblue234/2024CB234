#include "Controls.h"
#include "RobotExt.h"

void Controls::ControlsPeriodic(Drivetrain &swerve, Shooter &shooter, Intake &intake, Feeder &feeder, Elevator &elevator)
{
    // maybe an issue because I might be passing in a reference to a refernce
    // try making an instance variable a reference to fix this
    DriveControls(swerve);
    ShooterControls(shooter);
    IntakeControls(intake);
    ElevatorControls(shooter, elevator);
    FeederControls(shooter, elevator, feeder);
}

void Controls::DriveControls(const Drivetrain &swerve)
{
    if (gamepad.GetXButton() == true)
        swerve.SetFieldRelative(true);
    if (gamepad.GetBButton() == true)
        swerve.SetFieldRelative(false);
    
    if (gamepad.GetYButton() == true)
	    swerve.ResetGyroAngle();
    
    if (swerve.IsAlignmentOn()) 
		swerve.AlignSwerveDrive();
    else 
	{
        double rot = gamepad.GetRightX();
        if (gamepad.GetRightTriggerAxis() > 0.2) 
        {
            rot = limelight3.GetAprilTagOffset() / 360;
        }
		swerve.DriveWithInput(gamepad.GetLeftY(), gamepad.GetLeftX(), rot, gamepad.GetLeftStickButton());
	}
}

void Controls::ShooterControl(const Shooter &shooter)
{
    shooter.shootAtSpeaker = frc::SmartDashboard::GetBoolean("Shoot At Speaker?", shooter.shootAtSpeaker);

    if (gamepad.GetRightTriggerAxis() > 0.2)
    {
        if (shooter.shootAtSpeaker)
			shooter.ShootAtSpeaker();
        else 
			shooter.ShootAtAmp();
    }
    else if (gamepad.GetLeftBumper()) 
		shooter.IntakeFromSource();
    else
        shooter.SetShooterMotors(0.0);
}

void Controls::IntakeControls(const Intake &intake)
{
    if (gamepad.GetLeftTriggerAxis() > 0.2) 
		intake.IntakeFromGround();
    else 
		intake.SetIntakeMotor(0.0);
}

void Controls::ElevatorControls(const Shooter &shooter, const Elevator &elevator)
{
    shooter.shootAtSpeaker = frc::SmartDashboard::GetBoolean("Shoot At Speaker?", shooter.shootAtSpeaker);

    // Manual up - dpad up
    if (gamepad.GetPOV() > 315 && gamepad.GetPOV() < 45)
        elevator.SetElevatorMotors(elevator.GetElevatorSpeed());    
    // Manual down - dpad down
    else if (gamepad.GetPOV() < 215 && gamepad.GetPOV() > 135)
        elevator.SetElevatorMotors(-elevator.GetElevatorSpeed());
    else if (gamepad.GetRightTriggerAxis() > 0.2)
    {
        if (shooter.shootAtSpeaker)
		{
			// todo swerve.AlignToAprilTag();
			// is the swerve at position && is the elevator at position
			//elevator.AlignShooterToSpeaker()
            elevator.AlignShooterToSpeaker();
		}
        else 
		{
            elevator.SetElevatorMotorsPosition(elevator.GetShooterRevolutions() + );
        }
    }
}

void Controls::FeederControls(const Shooter &shooter, const Elevator &elevator, const Feeder &feeder)
{
    shooter.shootAtSpeaker = frc::SmartDashboard::GetBoolean("Shoot At Speaker?", shooter.shootAtSpeaker);

    if (gamepad.GetRightTriggerAxis() > 0.2)
    {
        if (shooter.shootAtSpeaker)
		{
			// todo swerve.AlignToAprilTag();
			// is the swerve at position && is the elevator at position
			//elevator.AlignShooterToSpeaker()
            bool atAlignment = Math.abs(limelight3.GetAprilTagOffset()) < 0.5 && Math.abs(elevator.getAlignmentDifference()) < 0.5;
			if (shooter.GetAverageRPM() >= shooter.GetSpeakerRPM() - 15 && atAlignment) 
                feeder.ShootAtSpeaker();
		}
        else 
			feeder.ShootAtAmp();
    }
    else if (gamepad.GetLeftBumper()) 
		feeder.IntakeFromSource();
    else if (gamepad.GetLeftTriggerAxis() > 0.2) 
		feeder.IntakeFromGround();
    else
        feeder.SetFeedMotor(0.0);

}