#pragma once

#include "rev/CANSparkMax.h"
#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <frc/Timer.h>
#include <frc/DigitalInput.h>

class Feeder : frc2::SubsystemBase
{
public:
    Feeder();
    void IntakeFromGround();
    void IntakeFromSource();
    void ShootAtSpeaker();
    void ShootAtAmp();
    void SensorControl();
    void SetFeedMotor(double power) { feedMotor.Set(power); };

    double GetGroundIntakeSpeed() { return groundIntakeSpeed; };
    double GetSourceIntakeSpeed() { return sourceIntakeSpeed; };
    double GetSpeakerShooterSpeed() { return speakerShooterSpeed; };
    double GetAmpShooterSpeed() { return ampShooterSpeed; };

    bool GetSensorInput() { return feedSensor.Get(); };

private:
    rev::CANSparkMax feedMotor{RobotMap::FEED_MOTOR_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};

    frc::DigitalInput feedSensor{RobotMap::FEED_SENSOR_ADDRESS};
    frc::Timer feedSensorTimer{};
    
    // Speeds should be from 0.0 - 1.0
    double groundIntakeSpeed = 0.5;
    double sourceIntakeSpeed = 0.5;
    double speakerShooterSpeed = 0.5;
    double ampShooterSpeed = 0.5;
};