#include "subsystems/Shooter.h"
#include <frc/smartdashboard/SmartDashboard.h>

Shooter::Shooter()
{
    shooter1.RestoreFactoryDefaults();
    shooter2.RestoreFactoryDefaults();

    shooter1PID.SetP(ShooterConstants::kShooterP);
    shooter1PID.SetI(ShooterConstants::kShooterI);
    shooter1PID.SetD(ShooterConstants::kShooterD);
    shooter1PID.SetFF(ShooterConstants::kShooterF);

    shooter2PID.SetP(ShooterConstants::kShooterP);
    shooter2PID.SetI(ShooterConstants::kShooterI);
    shooter2PID.SetD(ShooterConstants::kShooterD);
    shooter2PID.SetFF(ShooterConstants::kShooterF);

    frc::SmartDashboard::PutNumber("Shooter1 Power", 0.0);
    frc::SmartDashboard::PutNumber("Shooter2 Power", 0.0);

    shooterAngleEncoder.SetPositionOffset(ShooterConstants::SHOOTER_ANGLE_OFFSET);
    shooterAngleEncoder.SetDistancePerRotation(-360);
}

void Shooter::ShooterControl()
{
    power1 = frc::SmartDashboard::GetNumber("Shooter1 Power", power1);
    power2 = frc::SmartDashboard::GetNumber("Shooter2 Power", power2);

    SetShooterMotor1(power1);
    SetShooterMotor2(power2);

    frc::SmartDashboard::PutNumber("Shooter1 RPM", shooter1Encoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter1 RPM * Gear Ratio", shooter1Encoder.GetVelocity() * 50 / 30);
    frc::SmartDashboard::PutNumber("Shooter2 RPM", shooter2Encoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter2 RPM * Gear Ratio", shooter2Encoder.GetVelocity() * 28 / 30);

    frc::SmartDashboard::PutNumber("Shooter Encoder Count", shooterAngleEncoder.GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("Shooter Angle Degrees", GetShooterAngle());

    frc::SmartDashboard::PutNumber("Shooter1 Current", shooter1.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Shooter2 Current", shooter2.GetOutputCurrent());
}
