#pragma once

#include "Constants.h"
#include "frc/geometry/Pose2d.h"
#include "rev/CANSparkMax.h"
#include <frc2/command/SubsystemBase.h>
#include <frc/DutyCycleEncoder.h>
#include "subsystems/Limelight.h"

class Elevator : frc2::SubsystemBase 
{

public:
    Elevator(Limelight*);
    void Periodic() override;

    void AlignShooterToSpeaker();
    double CalculateSpeakerAngle();
    
    void SetElevatorMotorsPosition(double pos);
    void SetElevatorMotors(double power) { elevatorMotor1.Set(power); };

    double GetShooterAngle() { return shooterAngleEncoder.GetDistance(); };
    double GetShooterRevolutions() { return (double) shooterAngleEncoder.Get(); };
    double GetElevatorSpeed() { return elevatorSpeed; };
    double GetAlignmentDifference() { return alignmentDifference; };
    double GetAmpAngle() { return ampAngle; };
    
    void UpdateTelemetry();

private:
    rev::CANSparkMax elevatorMotor1{RobotMap::ELEVATOR_MOTOR1_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax elevatorMotor2{RobotMap::ELEVATOR_MOTOR2_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkPIDController elevatorPID = elevatorMotor1.GetPIDController();

    frc::DutyCycleEncoder shooterAngleEncoder{RobotMap::SHOOTER_ENCODER_ADDRESS};

    Limelight *limelight3;

    double elevatorSpeed = 0.5;
    double alignmentDifference = 0;
    // Should be a constant eventually
    double ampAngle = 22;
};