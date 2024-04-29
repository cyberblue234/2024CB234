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
#include "rev/CANSparkFlex.h"
#include "Constants.h"
#include "Controls.h"

class Shooter : frc2::SubsystemBase
{
public:
    Shooter();
    void Periodic() override;
    void ShooterControls();

    void ShootAtSpeaker();
    void ShootAtTrap();
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

    void SetShooterMotor1RPM(double rpm) { shooter1PID.SetReference(rpm, rev::CANSparkLowLevel::ControlType::kVelocity); };
    void SetShooterMotor2RPM(double rpm) { shooter2PID.SetReference(rpm, rev::CANSparkLowLevel::ControlType::kVelocity); };
    void StopMotors() { SetShooterMotor1(0.0); SetShooterMotor2(0.0); };

    static void SetSpeakerRPM(double rpm) { speakerRPM = rpm; };
    static void SetAmpRPM(double rpm) { ampRPM = rpm; };
    static void SetTrapRPM(double rpm) { trapRPM = rpm; };

    double GetAverageRPM() { return (GetShooter1RPM() + GetShooter2RPM()) / 2.0; };
    double GetShooter1RPM() { return shooter1Encoder.GetVelocity(); };
    double GetShooter2RPM() { return shooter2Encoder.GetVelocity(); };

    static double GetSpeakerRPM() { return speakerRPM; };
    static double GetAmpRPM() { return ampRPM; };
    static double GetTrapRPM() { return trapRPM; };
    double GetIntakeSpeed() { return intakeSpeed; };

private:
    rev::CANSparkFlex shooter1Motor{RobotMap::SHOOTER_MOTOR1_ADDRESS, rev::CANSparkFlex::MotorType::kBrushless};
    rev::CANSparkFlex shooter2Motor{RobotMap::SHOOTER_MOTOR2_ADDRESS, rev::CANSparkFlex::MotorType::kBrushless};
    rev::SparkRelativeEncoder shooter1Encoder = shooter1Motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::SparkRelativeEncoder shooter2Encoder = shooter2Motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::SparkPIDController shooter1PID = shooter1Motor.GetPIDController();
    rev::SparkPIDController shooter2PID = shooter2Motor.GetPIDController();

    static double speakerRPM;
    static double trapRPM;
    static double ampRPM;
    double intakeSpeed = 0.225;
};