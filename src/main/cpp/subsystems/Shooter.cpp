#include "subsystems/Shooter.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include "RobotExt.h"

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

    frc::SmartDashboard::PutNumber("Shooter1 Power", 0.0);
    frc::SmartDashboard::PutNumber("Shooter2 Power", 0.0);
    frc::SmartDashboard::PutNumber("Feed Power", 0.0);

    shooterAngleEncoder.SetPositionOffset(ShooterConstants::SHOOTER_ANGLE_OFFSET);
    shooterAngleEncoder.SetDistancePerRotation(-360);
}

void Shooter::ShooterControl()
{
    shooter1Power = frc::SmartDashboard::GetNumber("Shooter1 Power", shooter1Power);
    shooter2Power = frc::SmartDashboard::GetNumber("Shooter2 Power", shooter2Power);
    feedPower = frc::SmartDashboard::GetNumber("Feed Power", feedPower);

    if (gamePad.GetAButton())
    {
        SetShooterMotor1(shooter1Power);
        SetShooterMotor2(shooter2Power);
    }
    else
    {
        SetShooterMotor1(0.0);
        SetShooterMotor2(0.0);
    }

    if (gamePad.GetBButton())
        SetFeedMotor(feedPower);
    else
        SetFeedMotor(0.0);

    frc::SmartDashboard::PutNumber("Shooter1 RPM", shooter1Encoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter1 RPM * Gear Ratio", shooter1Encoder.GetVelocity() * 50 / 30);
    frc::SmartDashboard::PutNumber("Shooter2 RPM", shooter2Encoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter2 RPM * Gear Ratio", shooter2Encoder.GetVelocity() * 28 / 30);

    frc::SmartDashboard::PutNumber("Shooter Encoder Count", shooterAngleEncoder.GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("Shooter Angle Degrees", GetShooterAngle());

    frc::SmartDashboard::PutNumber("Shooter1 Current", shooter1Motor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Shooter2 Current", shooter2Motor.GetOutputCurrent());
    //frc::SmartDashboard::PutNumber("Feed Current", feedMotor.GetMotorOutputVoltage());
}