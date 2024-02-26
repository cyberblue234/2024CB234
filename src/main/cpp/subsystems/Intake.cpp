#include "subsystems/Intake.h"

Intake::Intake()
{
    intake.RestoreFactoryDefaults();
    frc::SmartDashboard::PutNumber("Intake Ground Speed", groundSpeed);
}

void Intake::Periodic()
{
    UpdateTelemetry();
}

void Intake::IntakeFromGround()
{
    groundSpeed = frc::SmartDashboard::GetNumber("Intake Ground Speed", groundSpeed);
    SetIntakeMotor(groundSpeed);
}

void Intake::UpdateTelemetry()
{
    frc::SmartDashboard::PutNumber("Intake RPM", intakeEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Intake Current", intake.GetOutputCurrent());
}