#include "subsystems/Shooter.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include "RobotExt.h"

#define VORTEX_MAX_RPM 6784

Shooter::Shooter()
{
    shooter1Motor.RestoreFactoryDefaults();
    shooter2Motor.RestoreFactoryDefaults();

    frc::SmartDashboard::PutNumber("Shooter P", ShooterConstants::kShooterP);
    frc::SmartDashboard::PutNumber("Shooter I", ShooterConstants::kShooterI);
    frc::SmartDashboard::PutNumber("Shooter D", ShooterConstants::kShooterD);
    frc::SmartDashboard::PutNumber("Shooter F", ShooterConstants::kShooterF);

    shooter1Motor.SetInverted(true);

    frc::SmartDashboard::PutNumber("Shooter Speaker RPM", speakerRPM);
    frc::SmartDashboard::PutNumber("Shooter Amp Speed", ampSpeed);
    frc::SmartDashboard::PutNumber("Shooter Intake Speed", intakeSpeed);
    
    frc::SmartDashboard::PutBoolean("Shoot At Speaker?", shootAtSpeaker);
}

void Shooter::Periodic()
{
    shooter1PID.SetP(frc::SmartDashboard::GetNumber("Shooter P", ShooterConstants::kShooterP));
    shooter1PID.SetI(frc::SmartDashboard::GetNumber("Shooter I", ShooterConstants::kShooterI));
    shooter1PID.SetD(frc::SmartDashboard::GetNumber("Shooter D", ShooterConstants::kShooterD));
    shooter1PID.SetFF(frc::SmartDashboard::GetNumber("Shooter F", ShooterConstants::kShooterF));

    shooter2PID.SetP(frc::SmartDashboard::GetNumber("Shooter P", ShooterConstants::kShooterP));
    shooter2PID.SetI(frc::SmartDashboard::GetNumber("Shooter I", ShooterConstants::kShooterI));
    shooter2PID.SetD(frc::SmartDashboard::GetNumber("Shooter D", ShooterConstants::kShooterD));
    shooter2PID.SetFF(frc::SmartDashboard::GetNumber("Shooter F", ShooterConstants::kShooterF));
}

void Shooter::ShootAtSpeaker(bool atAlignment)
{
    speakerRPM = frc::SmartDashboard::GetNumber("Shooter Speaker RPM", speakerRPM);
    SetShooterMotorsRPM(speakerRPM);
    if (GetAverageRPM() >= speakerRPM - 15 && atAlignment) feeder.ShootAtSpeaker();
}

void Shooter::ShootAtAmp()
{
    ampSpeed = frc::SmartDashboard::GetNumber("Shooter Amp Speed", ampSpeed);
    SetShooterMotors(ampSpeed);
    if (GetAverageRPM() >= (VORTEX_MAX_RPM / ampSpeed) - 25) feeder.ShootAtAmp();
}

void Shooter::IntakeFromSource()
{
    intakeSpeed = frc::SmartDashboard::GetNumber("Shooter Intake Speed", intakeSpeed);
    SetShooterMotors(-intakeSpeed);
    feeder.IntakeFromSource();
}

void Shooter::UpdateTelemetry()
{
    shootAtSpeaker = frc::SmartDashboard::GetBoolean("Shoot At Speaker?", shootAtSpeaker);

    frc::SmartDashboard::PutNumber("Shooter1 RPM", shooter1Encoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter1 RPM * Gear Ratio", shooter1Encoder.GetVelocity() * 50 / 30);
    frc::SmartDashboard::PutNumber("Shooter2 RPM", shooter2Encoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter2 RPM * Gear Ratio", shooter2Encoder.GetVelocity() * 28 / 30);

    frc::SmartDashboard::PutNumber("Shooter1 Current", shooter1Motor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Shooter2 Current", shooter2Motor.GetOutputCurrent());
}

frc2::CommandPtr Shooter::GetShooterCommand()
{
    return this->RunOnce(
        [this] {
            SetShooterMotorsRPM(this->GetSpeakerRPM());
        }
    );
}