#include "subsystems/Feeder.h"

Feeder::Feeder()
{
    feedMotor.RestoreFactoryDefaults();

    groundIntakeSpeed = frc::SmartDashboard::PutNumber("Feed Ground Speed", groundIntakeSpeed);
    sourceIntakeSpeed = frc::SmartDashboard::PutNumber("Feed Source Speed", sourceIntakeSpeed);
    speakerShooterSpeed = frc::SmartDashboard::PutNumber("Feed Speaker Speed", speakerShooterSpeed);
    ampShooterSpeed = frc::SmartDashboard::PutNumber("Feed Amp Speed", ampShooterSpeed);

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
    speakerShooterSpeed = frc::SmartDashboard::GetNumber("Feed Speaker Speed", speakerShooterSpeed);
    SetFeedMotor(0.85);
}

void Feeder::ShootAtAmp()
{
    ampShooterSpeed = frc::SmartDashboard::GetNumber("Feed Amp Speed", ampShooterSpeed);
    SetFeedMotor(0.85);
}

void Feeder::SensorControl()
{
    // Sensor does not detect a gamepiece
    if (feedSensor.Get() == false)
        feedMotorEncoder.SetPosition(0.0);
}