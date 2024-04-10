#include "subsystems/Intake.h"

Intake::Intake()
{
    intake.RestoreFactoryDefaults();
}

void Intake::Periodic()
{
    UpdateTelemetry();
}

void Controls::IntakeControls(bool elevatorAlignment, bool isNoteSecured)
{
    if (Controls::Intake() == true)
    {
        if (elevatorAlignment == true)
        {
            if (isNoteSecured == false)
            {
                IntakeFromGround();
                LED::LEDControls(LED::ControlMethods::kIntaking);
            }
            else
            {
                SetIntakeMotor(-0.1);
                LED::LEDControls(LED::ControlMethods::kNoteSecured);
            }
        }
        else
        {
            SetIntakeMotor(0.0);
            LED::LEDControls(LED::ControlMethods::kIntaking);
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