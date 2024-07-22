#include "subsystems/Shooter.h"

#define VORTEX_MAX_RPM 6784

Shooter::Shooter()
{
    shooter1Motor.RestoreFactoryDefaults();
    shooter2Motor.RestoreFactoryDefaults();

    shooter1Motor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    shooter2Motor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);

    shooter1Motor.SetInverted(true);

    frc::SmartDashboard::PutNumber("Shooter Speaker RPM", speakerRPM);
    frc::SmartDashboard::PutNumber("Shooter Amp RPM", ampRPM);
    frc::SmartDashboard::PutNumber("Shooter Intake Speed", intakeSpeed);

    shooter1PID.SetP(ShooterConstants::kShooterP);
    shooter1PID.SetI(ShooterConstants::kShooterI);
    shooter1PID.SetD(ShooterConstants::kShooterD);
    shooter1PID.SetFF(ShooterConstants::kShooterF);

    shooter2PID.SetP(ShooterConstants::kShooterP);
    shooter2PID.SetI(ShooterConstants::kShooterI);
    shooter2PID.SetD(ShooterConstants::kShooterD);
    shooter2PID.SetFF(ShooterConstants::kShooterF);
}

void Shooter::Periodic()
{
    UpdateTelemetry();
}

void Shooter::ShootAtSpeaker()
{
    SetShooterMotor1RPM(speakerRPM + 2000);
    SetShooterMotor2RPM(speakerRPM);
}

void Shooter::Pass()
{
    SetShooterMotor1RPM(passRPM + 2000);
    SetShooterMotor2RPM(passRPM);
}

void Shooter::ShootAtAmp()
{
    SetShooterMotor1RPM(ampRPM);
    SetShooterMotor2RPM(ampRPM);
}

void Shooter::IntakeFromSource()
{
    SetShooterMotors(-intakeSpeed);
}

void Shooter::UpdateTelemetry()
{
    frc::SmartDashboard::PutNumber("Shooter1 RPM", shooter1Encoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter2 RPM", shooter2Encoder.GetVelocity());

    frc::SmartDashboard::PutNumber("Shooter1 Current", shooter1Motor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Shooter2 Current", shooter2Motor.GetOutputCurrent());
}