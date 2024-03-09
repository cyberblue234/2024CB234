#include "subsystems/Feeder.h"

Feeder::Feeder()
{
    feedMotor.RestoreFactoryDefaults();

    feedSensorTimer.Start();
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