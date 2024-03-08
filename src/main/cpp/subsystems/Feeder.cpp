#include "subsystems/Feeder.h"

Feeder::Feeder()
{
    feedMotor.RestoreFactoryDefaults();

    feedSensorTimer.Start();
}

bool Feeder::IntakeFromGround()
{
    SensorControl();
    if (feedMotorEncoder.GetPosition() < 1)
    {
        SetFeedMotor(groundIntakeSpeed);
        return false;
    }
    else
    {
        SetFeedMotor(0.0);
        return true;
    }
}

bool Feeder::IntakeFromSource()
{
    SensorControl();
    if (feedMotorEncoder.GetPosition() > -1)
    {
        SetFeedMotor(-sourceIntakeSpeed);
        return false;
    }
    else
    {
        SetFeedMotor(0.0);
        return true;
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

void Feeder::SensorControl()
{
    // Sensor does not detect a gamepiece
    if (feedSensor.Get() == false)
        feedMotorEncoder.SetPosition(0.0);
}