#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <frc/DigitalInput.h>
#include "rev/CANSparkMax.h"
#include "Constants.h"

class Feeder : frc2::SubsystemBase
{
public:
    Feeder();
    bool IntakeFromGround();
    bool IntakeFromSource();
    void ShootAtSpeaker();
    void ShootAtAmp();
    void SensorControl();
    void Purge() { SetFeedMotor(-1.0); };
    void SetFeedMotor(double power) { feedMotor.Set(power); };

    double GetFeedMotorRPM() { return feedMotorEncoder.GetVelocity(); }
    double GetGroundIntakeSpeed() { return groundIntakeSpeed; };
    double GetSourceIntakeSpeed() { return sourceIntakeSpeed; };
    double GetSpeakerShooterSpeed() { return speakerShooterSpeed; };
    double GetAmpShooterSpeed() { return ampShooterSpeed; };

    bool GetSensorInput() { return feedSensor.Get(); };

private:
    rev::CANSparkMax feedMotor{RobotMap::FEED_MOTOR_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkRelativeEncoder feedMotorEncoder = feedMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

    frc::DigitalInput feedSensor{RobotMap::FEED_SENSOR_ADDRESS};
    frc::Timer feedSensorTimer{};

    // Speeds should be from 0.0 - 1.0
    double groundIntakeSpeed = 0.5;
    double sourceIntakeSpeed = 0.5;
    double speakerShooterSpeed = 0.5;
    double ampShooterSpeed = 0.5;
};