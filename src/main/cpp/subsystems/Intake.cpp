#include "subsystems/Intake.h"
#include <frc/smartdashboard/SmartDashboard.h>

Intake::Intake()
{
    intake.RestoreFactoryDefaults();
    frc::SmartDashboard::PutNumber("Intake Power", 0.0);
}

void Intake::IntakeControl()
{
    power = frc::SmartDashboard::GetNumber("Intake Power", power);

    SetIntakeMotor(power);
    frc::SmartDashboard::PutNumber("Intake RPM", intakeEncoder.GetVelocity());

    frc::SmartDashboard::PutNumber("Intake Current", intake.GetOutputCurrent());
}
