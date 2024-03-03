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

void Feeder::IntakeFromGround()
{
    //groundIntakeSpeed = frc::SmartDashboard::GetNumber("Feed Ground Speed", groundIntakeSpeed);

    SensorControl();
    if (feedMotorEncoder.GetPosition() < 4)
        SetFeedMotor(groundIntakeSpeed);
    else
        SetFeedMotor(0.0);
}

void Feeder::IntakeFromSource()
{
    //sourceIntakeSpeed = frc::SmartDashboard::GetNumber("Feed Source Speed", sourceIntakeSpeed);

    SensorControl();
    if (feedMotorEncoder.GetPosition() > -4)
        SetFeedMotor(-sourceIntakeSpeed);
    else
        SetFeedMotor(0.0);
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