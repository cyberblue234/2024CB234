#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include <frc2/command/StartEndCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Timer.h>
#include <frc/DigitalInput.h>
#include "rev/CANSparkMax.h"
#include "Constants.h"

class Shooter : frc2::SubsystemBase
{
public:
    Shooter();
    void Periodic() override;
    void ShootAtSpeaker();
    void ShootAtAmp();
    void IntakeFromSource();

    void UpdateTelemetry();

    void Purge() { SetShooterMotors(-1.0); };
    void SetShooterMotors(double power)
    {
        SetShooterMotor1(power);
        SetShooterMotor2(power);
    };
    void SetShooterMotor1(double power) { shooter1Motor.Set(power); };
    void SetShooterMotor2(double power) { shooter2Motor.Set(power); };
    void SetShooterMotorsRPM(double rpm)
    {
        SetShooterMotor1RPM(rpm);
        SetShooterMotor2RPM(rpm);
    };
    void SetShooterMotor1RPM(double rpm) { shooter1PID.SetReference(rpm, rev::CANSparkLowLevel::ControlType::kVelocity); };
    void SetShooterMotor2RPM(double rpm) { shooter2PID.SetReference(rpm, rev::CANSparkLowLevel::ControlType::kVelocity); };

    double GetAverageRPM() { return (GetShooter1RPM() + GetShooter2RPM()) / 2.0; };
    double GetShooter1RPM() { return shooter1Encoder.GetVelocity(); };
    double GetShooter2RPM() { return shooter2Encoder.GetVelocity(); };
    double GetSpeakerRPM() { return speakerRPM; };
    double GetAmpSpeed() { return ampSpeed; };
    double GetIntakeSpeed() { return intakeSpeed; };

private:
    rev::CANSparkMax shooter1Motor{RobotMap::SHOOTER_MOTOR1_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax shooter2Motor{RobotMap::SHOOTER_MOTOR2_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkRelativeEncoder shooter1Encoder = shooter1Motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::SparkRelativeEncoder shooter2Encoder = shooter2Motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::SparkPIDController shooter1PID = shooter1Motor.GetPIDController();
    rev::SparkPIDController shooter2PID = shooter2Motor.GetPIDController();

    double speakerRPM = 4500;
    double ampSpeed = 0.30;
    double intakeSpeed = 0.25;
};