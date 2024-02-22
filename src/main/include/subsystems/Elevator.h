#pragma once

#include "Constants.h"
#include "frc/geometry/Pose2d.h"
#include "rev/CANSparkMax.h"
#include <frc2/command/SubsystemBase.h>
#include <frc/DutyCycleEncoder.h>

class Elevator : frc2::SubsystemBase 
{

public:
    Elevator();
    void Periodic() override;

    void AlignShooterToSpeaker();
    double CalculateSpeakerAngle();
    
    void SetElevatorMotorsPosition(double pos) { elevatorPID.SetReference(pos, rev::CANSparkLowLevel::ControlType::kPosition); };
    void SetElevatorMotors(double power) { elevatorMotor1.Set(power); };

    double GetShooterAngle() { return shooterAngleEncoder.GetDistance(); };
    double GetShooterRevolutions() { return (double) shooterAngleEncoder.Get(); };
    
    void UpdateTelemetry();

private:
    rev::CANSparkMax elevatorMotor1{RobotMap::ELEVATOR_MOTOR1_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax elevatorMotor2{RobotMap::ELEVATOR_MOTOR2_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkPIDController elevatorPID = elevatorMotor1.GetPIDController();

    frc::DutyCycleEncoder shooterAngleEncoder{RobotMap::SHOOTER_ENCODER_ADDRESS};
};