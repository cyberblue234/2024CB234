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

    SetSpeakerRPM(4500);
    SetTrapRPM(3000);
    SetAmpRPM(2000);
}

void Shooter::Periodic()
{
    UpdateTelemetry();
}

void Shooter::ShooterControls()
{
    if (Controls::AmpMain() == true)
    {
        SetAmpRPM(2100);
    }
    if (Controls::Amp2() == true) 
    {
        SetAmpRPM(2200);
    }
    else if (Controls::Amp3() == true) 
    {
        SetAmpRPM(2000);
    }
    else if (Controls::Amp4() == true)
    {
        SetAmpRPM(1900);
    }

    if (Controls::ShooterMotors() == true)
    {
        if (Controls::AmpShot() == true)
        {
            ShootAtAmp();
        }
        else if (Controls::TrapShot() == true)
        {
            ShootAtTrap();
        }
        else
        {
            ShootAtSpeaker();
        }
    }
    else if (Controls::SourceIntake() == true)
    {
        IntakeFromSource();
    }
    else if (Controls::Purge() == true)
    {
        Purge();
    }
    else
    {
        StopMotors();
    }
}

void Shooter::ShootAtSpeaker()
{
    SetShooterMotor1RPM(speakerRPM + 2000);
    SetShooterMotor2RPM(speakerRPM);
}

void Shooter::ShootAtTrap()
{
    SetShooterMotor1RPM(trapRPM + 2000);
    SetShooterMotor2RPM(trapRPM);
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