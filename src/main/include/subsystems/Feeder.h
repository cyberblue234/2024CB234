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
    void Periodic() override;
    void IntakeFromGround();
    void IntakeFromSource();
    void ShootAtSpeaker();
    void ShootAtAmp();
    bool IsNoteSecured() { return GetTopSensorInput() && GetBottomSensorInput(); };
    void Purge() { SetFeedMotor(-1.0); };
    void SetFeedMotor(double power) { feedMotor.Set(power); };
    void StopMotor() { SetFeedMotor(0.0); };

    double GetFeedMotorRPM() { return feedMotorEncoder.GetVelocity(); }
    double GetGroundIntakeSpeed() { return groundIntakeSpeed; };
    double GetSourceIntakeSpeed() { return sourceIntakeSpeed; };
    double GetSpeakerShooterSpeed() { return speakerShooterSpeed; };
    double GetAmpShooterSpeed() { return ampShooterSpeed; };

    bool GetTopSensorInput() { return topFeedSensor.Get(); };
    bool GetBottomSensorInput() { return bottomFeedSensor.Get(); };

private:
    rev::CANSparkMax feedMotor{RobotMap::FEED_MOTOR_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkRelativeEncoder feedMotorEncoder = feedMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

    frc::DigitalInput topFeedSensor{RobotMap::TOP_FEED_SENSOR_ADDRESS};
    frc::DigitalInput bottomFeedSensor{RobotMap::BOTTOM_FEED_SENSOR_ADDRESS};
    frc::Timer feedSensorTimer{};

    // Speeds should be from 0.0 - 1.0
    double groundIntakeSpeed = 0.35;
    double sourceIntakeSpeed = 0.35;
    double speakerShooterSpeed = 0.5;
    double ampShooterSpeed = 0.5;
};