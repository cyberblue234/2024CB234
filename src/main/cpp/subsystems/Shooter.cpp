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

    shooter1PID.SetP(ShooterConstants::kShooterP);
    shooter1PID.SetI(ShooterConstants::kShooterI);
    shooter1PID.SetD(ShooterConstants::kShooterD);
    shooter1PID.SetFF(ShooterConstants::kShooterF);

    shooter2PID.SetP(ShooterConstants::kShooterP);
    shooter2PID.SetI(ShooterConstants::kShooterI);
    shooter2PID.SetD(ShooterConstants::kShooterD);
    shooter2PID.SetFF(ShooterConstants::kShooterF);

    shooter1Motor.SetInverted(true);

    frc::SmartDashboard::PutNumber("Shooter Speaker Speed", speakerSpeed);
    frc::SmartDashboard::PutNumber("Shooter Amp Speed", ampSpeed);
    frc::SmartDashboard::PutNumber("Shooter Intake Speed", intakeSpeed);
    
    frc::SmartDashboard::PutBoolean("Shoot At Speaker?", shootAtSpeaker);

    shooterAngleEncoder.SetPositionOffset(ShooterConstants::SHOOTER_ANGLE_OFFSET);
    shooterAngleEncoder.SetDistancePerRotation(-360);
}

void Shooter::ShootAtSpeaker()
{
    speakerSpeed = frc::SmartDashboard::GetNumber("Shooter Speaker Speed", speakerSpeed);
    SetShooterMotors(speakerSpeed);
    if (GetAverageRPM() >= (VORTEX_MAX_RPM / speakerSpeed) - 100) feeder.ShootAtSpeaker();
}

void Shooter::ShootAtAmp()
{
    ampSpeed = frc::SmartDashboard::GetNumber("Shooter Amp Speed", ampSpeed);
    SetShooterMotors(ampSpeed);
    if (GetAverageRPM() >= (VORTEX_MAX_RPM / ampSpeed) - 100) feeder.ShootAtAmp();
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

    frc::SmartDashboard::PutNumber("Shooter Encoder Count", shooterAngleEncoder.GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("Shooter Angle Degrees", GetShooterAngle());

    frc::SmartDashboard::PutNumber("Shooter1 Current", shooter1Motor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Shooter2 Current", shooter2Motor.GetOutputCurrent());
}

frc2::CommandPtr Shooter::GetShooterCommand()
{
    return this->RunOnce(
        [this] {
            SetShooterMotor1(this->GetSpeakerSpeed());
            SetShooterMotor2(this->GetSpeakerSpeed());
        }
    );
}