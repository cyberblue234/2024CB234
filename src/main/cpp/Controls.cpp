#include "Controls.h"

bool noteInBot = false;

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
    SetSelectedRotaryIndex(AnalogToRotaryIndex(controlBoard.GetX()));
    frc::SmartDashboard::PutNumber("Control Board X", controlBoard.GetX());

    frc::SmartDashboard::PutNumber("Rotary Index", GetSelectedRotaryIndex());
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

    if (gamepad.GetRightBumper()) 
        swerve->SetPIDFs();

    if (swerve->IsAlignmentOn())
        swerve->AlignSwerveDrive();
    else if (controlBoard.GetRawButton(ControlBoardConstants::ANCHOR))
        swerve->SetAnchorState();
    else
    {
        double rot = swerve->RotationControl(gamepad.GetRightX(), 
                                controlBoard.GetRawButton(ControlBoardConstants::SHOOT) 
                                && GetSelectedRotaryIndex() != ControlBoardConstants::MANUAL_SCORE);
        swerve->DriveWithInput(gamepad.GetLeftY(), gamepad.GetLeftX(), rot, gamepad.GetRightTriggerAxis() > 0.2);
    }
}

void Controls::ShooterControls()
{
    switch (GetSelectedRotaryIndex())
    {
        case 5:
            shooter->SetAmpRPM(2200);
            break;
        case 6:
            shooter->SetAmpRPM(2100);
            break;
        case 7:
            shooter->SetAmpRPM(1900);
            break;
        case 8:
            shooter->SetAmpRPM(1800);
            break;
        default:
            shooter->SetAmpRPM(1700);
            break;
    }
    if (controlBoard.GetRawButton(ControlBoardConstants::SHOOTER_MOTORS))
    {
        if (GetSelectedRotaryIndex() == ControlBoardConstants::POS_AMP 
        || GetSelectedRotaryIndex() == 5 
        || GetSelectedRotaryIndex() == 6 
        || GetSelectedRotaryIndex() == 7 
        || GetSelectedRotaryIndex() == 8)
            shooter->ShootAtAmp();
        else
            shooter->ShootAtSpeaker();
    }
    else if (controlBoard.GetRawButton(ControlBoardConstants::SOURCE_INTAKE))
        shooter->IntakeFromSource();
    else if (controlBoard.GetRawButton(ControlBoardConstants::PURGE))
        shooter->Purge();
    else
        shooter->SetShooterMotors(0.0);
}

void Controls::IntakeControls()
{
    if (controlBoard.GetRawButton(ControlBoardConstants::GROUND_INTAKE))
    {
        if (noteInBot == false)
            intake->IntakeFromGround();
        else
            intake->SetIntakeMotor(-0.1);
    }
    else if (controlBoard.GetRawButton(ControlBoardConstants::PURGE))
        intake->Purge();
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
    if (controlBoard.GetRawButton(ControlBoardConstants::ELEVATOR_UP))
    {
        if (elevator->GetElevator1TopLimit() == false && elevator->GetElevator1Encoder() < elevator->GetHardEncoderLimit())
            elevator->SetElevator1Motor(elevator->GetElevatorSpeed());
        else    
            elevator->SetElevator1Motor(0.0);
        if (elevator->GetElevator2TopLimit() == false && elevator->GetElevator2Encoder() < elevator->GetHardEncoderLimit())
            elevator->SetElevator2Motor(elevator->GetElevatorSpeed());
        else    
            elevator->SetElevator2Motor(0.0);
    }
    // Manual down - dpad down
    else if (controlBoard.GetRawButton(ControlBoardConstants::ELEVATOR_DOWN))
    {
        if (elevator->GetElevator1BottomLimit() == false)
            elevator->SetElevator1Motor(-elevator->GetElevatorSpeed());
        else    
            elevator->SetElevator1Motor(0.0);
        if (elevator->GetElevator2BottomLimit() == false)
            elevator->SetElevator2Motor(-elevator->GetElevatorSpeed());
        else    
            elevator->SetElevator2Motor(0.0);
    }
    else if (controlBoard.GetRawButton(ControlBoardConstants::GROUND_INTAKE))
    {
        elevator->ElevatorControl(elevator->GetIntakeAngle());
    }
    else if (controlBoard.GetRawButton(ControlBoardConstants::SOURCE_INTAKE))
    {
        elevator->ElevatorControl(elevator->GetIntakeAngle());
    }
    else if (controlBoard.GetRawButton(ControlBoardConstants::SHOOT) && GetSelectedRotaryIndex() != ControlBoardConstants::MANUAL_SCORE)
    {
    
        double angle;
        switch (GetSelectedRotaryIndex())
        {
            case ControlBoardConstants::AUTO_SCORE:
                if (limelight3->GetTargetValid() == 1)
                    angle = elevator->CalculateSpeakerAngle();
                else
                    angle = elevator->GetIntakeAngle();
                break;
            case ControlBoardConstants::POS_MID:
                angle = elevator->GetMidAngle();
                break;
            case ControlBoardConstants::POS_STAGE:
                angle = elevator->GetStageAngle();
                break;
            case ControlBoardConstants::POS_AMP:
                angle = elevator->GetAmpAngle();
                break;
            case 5:
                angle = elevator->GetAmpAngle();
                break;
            case 6:
                angle = elevator->GetAmpAngle();
                break;
            case 7:
                angle = elevator->GetAmpAngle();
                break;
            case 8:
                angle = elevator->GetAmpAngle();
                break;
            // Default is the close angle
            default:
                angle = elevator->GetCloseAngle();
        }

        elevator->ElevatorControl(angle);
    }
    else
        elevator->SetElevatorMotors(0.0);
}

void Controls::FeederControls()
{
    if (controlBoard.GetRawButton(ControlBoardConstants::SHOOT))
    {
        if (GetSelectedRotaryIndex() == ControlBoardConstants::POS_AMP
        || GetSelectedRotaryIndex() == 5 
        || GetSelectedRotaryIndex() == 6 
        || GetSelectedRotaryIndex() == 7 
        || GetSelectedRotaryIndex() == 8)
            feeder->ShootAtAmp();
        else if (GetSelectedRotaryIndex() != ControlBoardConstants::MANUAL_SCORE)
        {
            bool atAlignment = abs(limelight3->GetAprilTagOffset()) < 1.0 && abs(elevator->GetAlignmentDifference()) < 0.5;
            if (shooter->GetShooter1RPM() >= shooter->GetSpeakerRPM() - 100 && atAlignment)
                feeder->ShootAtSpeaker();
        }
        else
            feeder->ShootAtSpeaker();
    }
    else if (controlBoard.GetRawButton(ControlBoardConstants::SOURCE_INTAKE))
    {
        if (feeder->IntakeFromSource()) RumbleGamepad();
        else StopRumble();
    }
    else if (controlBoard.GetRawButton(ControlBoardConstants::GROUND_INTAKE))
    {
        if (feeder->IntakeFromGround()) 
        {
            RumbleGamepad();
            noteInBot = true;
        }
        else
        {
            StopRumble();
            noteInBot = false;
        }
    }
    else if (controlBoard.GetRawButton(ControlBoardConstants::PURGE))
        feeder->Purge();
    else
    {
        feeder->SetFeedMotor(0.0);
        StopRumble();
    }
}