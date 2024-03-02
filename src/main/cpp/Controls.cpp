#include "Controls.h"

Controls::Controls(Drivetrain *swerve, Shooter *shooter, Intake *intake, Elevator *elevator, Feeder *feeder, Limelight *limelight3)
{
    this->swerve = swerve;
    this->shooter = shooter;
    this->intake = intake;
    this->feeder = feeder;
    this->elevator = elevator;
    this->limelight3 = limelight3;
}

void Controls::Periodic()
{
    shooter->shootAtSpeaker = frc::SmartDashboard::GetBoolean("Shoot At Speaker?", shooter->shootAtSpeaker);
    DriveControls();
    ShooterControls();
    IntakeControls();
    ElevatorControls();
    FeederControls();
}

void Controls::DriveControls()
{
    if (gamepad.GetXButton() == true)
        swerve->SetFieldRelative(true);
    if (gamepad.GetBButton() == true)
        swerve->SetFieldRelative(false);

    if (gamepad.GetYButton() == true)
    {
        swerve->ResetGyroAngle();
        if (limelight3->GetTargetValid() == 1)
            swerve->ResetPose(limelight3->GetRobotPose());
    }

    //if (gamepad.GetRightBumper()) swerve->SetPIDFs();

    if (swerve->IsAlignmentOn())
        swerve->AlignSwerveDrive();
    else
    {
        double rot = swerve->RotationControl(-gamepad.GetRightX(), gamepad.GetRightTriggerAxis() > 0.2);
        swerve->DriveWithInput(gamepad.GetLeftY(), gamepad.GetLeftX(), rot, gamepad.GetLeftStickButton());
    }
}

void Controls::ShooterControls()
{
    if (gamepad.GetRightTriggerAxis() > 0.2)
    {
        if (shooter->shootAtSpeaker)
            shooter->ShootAtSpeaker();
        else
            shooter->ShootAtAmp();
    }
    else if (gamepad.GetLeftBumper())
        shooter->IntakeFromSource();
    else
        shooter->SetShooterMotors(0.0);
}

void Controls::IntakeControls()
{
    if (gamepad.GetLeftTriggerAxis() > 0.2)
        intake->IntakeFromGround();
    else
        intake->SetIntakeMotor(0.0);
}

void Controls::ElevatorControls()
{
    if (gamepad.GetPOV() == 270) 
    {
        elevator->SetElevator1Motor(gamepad.GetRightBumper() ? -0.5 : 0.5);
        return;
    }
    else if (gamepad.GetPOV() == 90) 
    {
        elevator->SetElevator2Motor(gamepad.GetRightBumper() ? -0.5 : 0.5);
        return;
    }

    // Manual up - dpad up
    if (gamepad.GetPOV() == 0)
    {
        if (!elevator->GetElevator1TopLimit())
            elevator->SetElevator1Motor(elevator->GetElevatorSpeed());
        else    
            elevator->SetElevator1Motor(0.0);
        if (!elevator->GetElevator2TopLimit())
            elevator->SetElevator2Motor(elevator->GetElevatorSpeed());
        else    
            elevator->SetElevator2Motor(0.0);
    }
    // Manual down - dpad down
    else if (gamepad.GetPOV() == 180)
    {
        if (!elevator->GetElevator1BottomLimit())
            elevator->SetElevator1Motor(-elevator->GetElevatorSpeed());
        else    
            elevator->SetElevator1Motor(0.0);
        if (!elevator->GetElevator2BottomLimit())
            elevator->SetElevator2Motor(-elevator->GetElevatorSpeed());
        else    
            elevator->SetElevator2Motor(0.0);
    }
    // Align to either speaker or amp
    // else if (gamepad.GetRightTriggerAxis() > 0.2)
    // {
    //     if (shooter->shootAtSpeaker)
    //     {
    //         elevator->AlignShooterToSpeaker();
    //     }
    //     else
    //     {
    //         elevator->SetElevatorMotorsPosition(elevator->GetShooterRevolutions() + (elevator->GetAmpAngle() / 360));
    //     }
    // }
    else
        elevator->SetElevatorMotors(0.0);
}

void Controls::FeederControls()
{
    if (gamepad.GetRightTriggerAxis() > 0.2)
    {
        if (shooter->shootAtSpeaker)
        {
            bool atAlignment = abs(limelight3->GetAprilTagOffset()) < 1.0; //&& abs(elevator->GetAlignmentDifference()) < 0.5;
            if (shooter->GetAverageRPM() >= shooter->GetSpeakerRPM() - 100)// && atAlignment)
                feeder->ShootAtSpeaker();
        }
        else if (shooter->GetAverageRPM() >= 2000)
            feeder->ShootAtAmp();
    }
    else if (gamepad.GetLeftBumper())
        feeder->IntakeFromSource();
    else if (gamepad.GetLeftTriggerAxis() > 0.2)
        feeder->IntakeFromGround();
    else
        feeder->SetFeedMotor(0.0);
}