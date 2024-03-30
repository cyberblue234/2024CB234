#include "Controls.h"

Controls::Controls(Drivetrain *swerve, Shooter *shooter, Intake *intake, Elevator *elevator, Feeder *feeder, Limelight *limelight3, ctre::phoenix6::Orchestra *orchestra)
{
    this->swerve = swerve;
    this->shooter = shooter;
    this->intake = intake;
    this->feeder = feeder;
    this->elevator = elevator;
    this->limelight3 = limelight3;
    this->orchestra = orchestra;
}

void Controls::Periodic()
{
    SetSelectedRotaryIndex(AnalogToRotaryIndex(controlBoard.GetX()));
    
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

    if (swerve->IsAlignmentOn())
        swerve->AlignSwerveDrive();
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
        case ControlBoardConstants::POS_AMP_2:
            shooter->SetAmpRPM(2200);
            break;
        case ControlBoardConstants::POS_AMP_3:
            shooter->SetAmpRPM(2000);
            break;
        case ControlBoardConstants::POS_AMP_4:
            shooter->SetAmpRPM(1900);
            break;
        default:
            shooter->SetAmpRPM(2100);
            break;
    }
    if (controlBoard.GetRawButton(ControlBoardConstants::SHOOTER_MOTORS))
    {
        if (GetSelectedRotaryIndex() == ControlBoardConstants::POS_AMP_MAIN
        || GetSelectedRotaryIndex() == ControlBoardConstants::POS_AMP_2
        || GetSelectedRotaryIndex() == ControlBoardConstants::POS_AMP_3
        || GetSelectedRotaryIndex() == ControlBoardConstants::POS_AMP_4
        || GetSelectedRotaryIndex() == ControlBoardConstants::MANUAL_AMP)
            shooter->ShootAtAmp();
        else
            shooter->ShootAtSpeaker();
    }
    else if (controlBoard.GetRawButton(ControlBoardConstants::SOURCE_INTAKE))
        shooter->IntakeFromSource();
    else if (controlBoard.GetRawButton(ControlBoardConstants::PURGE))
        shooter->Purge();
    else
        shooter->StopMotors();
}

void Controls::IntakeControls()
{
    if (controlBoard.GetRawButton(ControlBoardConstants::GROUND_INTAKE) && elevator->AtSetpoint())
    {
        if (feeder->IsNoteSecured() == false)
            intake->IntakeFromGround();
        else
            intake->SetIntakeMotor(-0.1);
    }
    else if (controlBoard.GetRawButton(ControlBoardConstants::PURGE))
        intake->Purge();
    else
        intake->StopMotor();
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
    else if (controlBoard.GetRawButton(ControlBoardConstants::ELEVATOR_UP))
    {
        elevator->ElevatorControl(elevator->GetElevatorSpeed(), Elevator::ControlMethods::Speed);
    }
    else if (controlBoard.GetRawButton(ControlBoardConstants::ELEVATOR_DOWN))
    {
        elevator->ElevatorControl(-elevator->GetElevatorSpeed(), Elevator::ControlMethods::Speed);
    }
    else if (controlBoard.GetRawButton(ControlBoardConstants::GROUND_INTAKE) || controlBoard.GetRawButton(ControlBoardConstants::SOURCE_INTAKE))
    {
        elevator->ElevatorControl(elevator->GetIntakeAngle(), Elevator::ControlMethods::Position);
    }
    else if (controlBoard.GetRawButton(ControlBoardConstants::SHOOTER_MOTORS) 
    && GetSelectedRotaryIndex() != ControlBoardConstants::MANUAL_SCORE && GetSelectedRotaryIndex() != ControlBoardConstants::MANUAL_AMP)
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
            case ControlBoardConstants::POS_AMP_MAIN:
                angle = elevator->GetAmpAngle();
                break;
            case ControlBoardConstants::POS_AMP_2:
                angle = elevator->GetAmpAngle();
                break;
            case ControlBoardConstants::POS_AMP_3:
                angle = elevator->GetAmpAngle();
                break;
            case ControlBoardConstants::POS_AMP_4:
                angle = elevator->GetAmpAngle();
                break;
            // Default is the close angle
            default:
                angle = elevator->GetCloseAngle();
        }
        frc::SmartDashboard::PutNumber("Desired Elevator Angle", angle);
        elevator->ElevatorControl(angle, Elevator::ControlMethods::Position);
    }
    else
    {
        if (controlBoard.GetRawButton(ControlBoardConstants::AUTO_ELEVATOR_DOWN) == true)
        {
            elevator->ElevatorControl(-elevator->GetElevatorSpeed(), Elevator::ControlMethods::Speed);
        }
        else
        {
            elevator->StopMotors();
        }
        
    }
}

void Controls::FeederControls()
{
    if (controlBoard.GetRawButton(ControlBoardConstants::SHOOT))
    {
        if (GetSelectedRotaryIndex() == ControlBoardConstants::POS_AMP_MAIN
        || GetSelectedRotaryIndex() == ControlBoardConstants::POS_AMP_2
        || GetSelectedRotaryIndex() == ControlBoardConstants::POS_AMP_3
        || GetSelectedRotaryIndex() == ControlBoardConstants::POS_AMP_4
        || GetSelectedRotaryIndex() == ControlBoardConstants::MANUAL_AMP)
            feeder->ShootAtAmp();
        else if (GetSelectedRotaryIndex() != ControlBoardConstants::MANUAL_SCORE)
        {
            bool swerveAlignment = swerve->AtSetpoint();
            bool elevatorAlignment = elevator->AtSetpoint();
            bool atAlignment = swerveAlignment && elevatorAlignment;
            if (shooter->GetShooter1RPM() >= shooter->GetSpeakerRPM() - 100 && atAlignment)
                feeder->ShootAtSpeaker();
        }
        else
            feeder->ShootAtSpeaker();
    }
    else if (controlBoard.GetRawButton(ControlBoardConstants::SOURCE_INTAKE))
    {
        feeder->IntakeFromSource();
        if (feeder->IsNoteSecured() == true) RumbleGamepad();
        else StopRumble();
    }
    else if (controlBoard.GetRawButton(ControlBoardConstants::GROUND_INTAKE))
    {
        feeder->IntakeFromGround();
        if (feeder->IsNoteSecured() == true) RumbleGamepad();
        else StopRumble();
    }
    else if (controlBoard.GetRawButton(ControlBoardConstants::PURGE))
        feeder->Purge();
    else
    {
        feeder->StopMotor();
        StopRumble();
    }
}