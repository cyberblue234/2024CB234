#include "subsystems/Feeder.h"

Feeder::Feeder()
{
    feedMotor.RestoreFactoryDefaults();

    feedSensorTimer.Start();
}

void Feeder::Periodic()
{
    frc::SmartDashboard::PutBoolean("Is Note Secured", IsNoteSecured());
}


void Feeder::FeederControls(bool swerveAlignment, bool elevatorAlignment, double averageShooterRPM)
{
    if (Controls::Shoot())
    {
        if (Controls::ShootAtAmp() == true)
        {
            ShootAtAmp();
        }
        else if (Controls::AutoScore() == true || Controls::Speaker() == true || Controls::Trap() == true)
        {
            bool atAlignment = swerveAlignment && elevatorAlignment;
            bool rpmAtSpeed;
            if (Controls::Trap() == true)
            {
                rpmAtSpeed = averageShooterRPM >= Shooter::GetTrapRPM() - 50;
            }
            else
            {
                rpmAtSpeed = averageShooterRPM >= Shooter::GetSpeakerRPM() - 100;
            }
            
            if (rpmAtSpeed == true && atAlignment) == true
            {
                ShootAtSpeaker();
            }
        }
        else if (Controls::ManualScore() == true || Controls::Mid() == true)
        {
            ShootAtSpeaker();
        }
    }
    else if (Controls::SourceIntake())
    {
        IntakeFromSource();
        if (IsNoteSecured() == true) Controls::RumbleGamepad();
        else Controls::StopRumble();
    }
    else if (Controls::GroundIntake())
    {
        IntakeFromGround();
        if (IsNoteSecured() == true) Controls::RumbleGamepad();
        else Controls::StopRumble();
    }
    else if (Controls::Purge())
    {
        Purge();
    }
    else
    {
        StopMotor();
        Controls::StopRumble();
    }
}

void Feeder::IntakeFromGround()
{
    if (IsNoteSecured() == false)
    {
        SetFeedMotor(groundIntakeSpeed);
    }
    else
    {
        SetFeedMotor(0.0);
    }
}

void Feeder::IntakeFromSource()
{
    if (IsNoteSecured() == false)
    {
        SetFeedMotor(-sourceIntakeSpeed);
    }
    else
    {
        SetFeedMotor(0.0);
    }
}

void Feeder::ShootAtSpeaker()
{
    SetFeedMotor(0.85);
}

void Feeder::ShootAtAmp()
{
    SetFeedMotor(0.85);
}