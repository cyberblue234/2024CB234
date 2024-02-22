#include "subsystems/Intake.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>

Intake::Intake()
{
    intake.RestoreFactoryDefaults();
    frc::SmartDashboard::PutNumber("Intake Ground Speed", 0.0);
}

void Intake::Periodic()
{
    UpdateTelemetry();
}

void Intake::IntakeFromGround()
{
    groundSpeed = frc::SmartDashboard::GetNumber("Intake Ground Speed", groundSpeed);
    SetIntakeMotor(groundSpeed);
    feeder.IntakeFromGround();
}

void Intake::UpdateTelemetry()
{
    frc::SmartDashboard::PutNumber("Intake RPM", intakeEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Intake Current", intake.GetOutputCurrent());
}

frc2::CommandPtr Intake::GetIntakeCommand()
{
    return this->RunOnce(
        [this] {
            SetIntakeMotor(this->GetGroundSpeed());
        }
    );
}