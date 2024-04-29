#include "subsystems/Intake.h"

Intake::Intake()
{
    intake.RestoreFactoryDefaults();
}

void Intake::Periodic()
{
    UpdateTelemetry();
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