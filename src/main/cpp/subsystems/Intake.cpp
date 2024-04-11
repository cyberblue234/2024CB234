#include "subsystems/Intake.h"

Intake::Intake(LED *candle)
{
    intake.RestoreFactoryDefaults();
    
    this->candle = candle;
}

void Intake::Periodic()
{
    UpdateTelemetry();
}

void Intake::IntakeControls(bool elevatorAlignment, bool isNoteSecured)
{
    if (Controls::GroundIntake() == true)
    {
        if (elevatorAlignment == true)
        {
            if (isNoteSecured == false)
            {
                IntakeFromGround();
                candle->LEDControls(LED::ControlMethods::kIntaking);
            }
            else
            {
                SetIntakeMotor(-0.1);
                candle->LEDControls(LED::ControlMethods::kNoteSecured);
            }
        }
        else
        {
            SetIntakeMotor(0.0);
            candle->LEDControls(LED::ControlMethods::kIntaking);
        }
    }
    else if (Controls::Purge())
    {
        Purge();
    }
    else
    {
        StopMotor();
    }
}

void Intake::IntakeFromGround()
{
    SetIntakeMotor(groundSpeed);
}

void Intake::UpdateTelemetry()
{
    frc::SmartDashboard::PutNumber("Intake RPM", intakeEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Intake Current", intake.GetOutputCurrent());
}