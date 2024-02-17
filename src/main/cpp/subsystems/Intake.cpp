#include "subsystems/Intake.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include "RobotExt.h"

Intake::Intake()
{
    intake.RestoreFactoryDefaults();
    frc::SmartDashboard::PutNumber("Intake Power", 0.0);
}

void Intake::IntakeControl()
{
    power = frc::SmartDashboard::GetNumber("Intake Power", power);
    if (gamePad.GetXButton())
    {
        SetIntakeMotor(power);
    }
    else
        SetIntakeMotor(0.0);
    frc::SmartDashboard::PutNumber("Intake RPM", intakeEncoder.GetVelocity());

    frc::SmartDashboard::PutNumber("Intake Current", intake.GetOutputCurrent());
}
