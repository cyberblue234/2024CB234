#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "frc/geometry/Pose2d.h"
#include <frc/DutyCycleEncoder.h>
#include "rev/CANSparkMax.h"
#include "subsystems/Limelight.h"
#include "Constants.h"
#include <numbers>

class Elevator : frc2::SubsystemBase
{

public:
    Elevator(Limelight *);
    void Periodic() override;

    void AlignShooterToSpeaker();
    double CalculateSpeakerAngle();

    void SetElevatorMotorsPosition(double pos) 
    { 
        SetElevator1MotorPosition(pos);
        SetElevator2MotorPosition(pos);
    };
    void SetElevator1MotorPosition(double pos) { elevator1PID.SetReference(pos, rev::CANSparkLowLevel::ControlType::kPosition); };
    void SetElevator2MotorPosition(double pos) { elevator2PID.SetReference(pos, rev::CANSparkLowLevel::ControlType::kPosition); };
    
    void SetElevatorMotors(double power) 
    { 
        SetElevator1Motor(power); 
        SetElevator2Motor(power); 
    };
    void SetElevator1Motor(double power) { elevator1Motor.Set(power); };
    void SetElevator2Motor(double power) { elevator2Motor.Set(power); };

    bool IsElevator1OutOfBounds() { return (double) elevator1Encoder.GetPosition() >= elevator1TopLimit 
                                        || (double) elevator1Encoder.GetPosition() <= elevator1BottomLimit; };
    bool IsElevator2OutOfBounds() { return (double) elevator2Encoder.GetPosition() >= elevator2TopLimit 
                                        || (double) elevator2Encoder.GetPosition() <= elevator2BottomLimit; };
    
    double GetShooterAngle() { return shooterAngleEncoder.GetDistance(); };
    double GetShooterRevolutions() { return (double) shooterAngleEncoder.Get(); };
    double GetElevatorSpeed() { return elevatorSpeed; };
    double GetAlignmentDifference() { return alignmentDifference; };
    double GetAmpAngle() { return ampAngle; };
    double GetIntakeAngle() { return intakeAngle; };

    void UpdateTelemetry();

private:
    rev::CANSparkMax elevator1Motor{RobotMap::ELEVATOR_MOTOR1_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax elevator2Motor{RobotMap::ELEVATOR_MOTOR2_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkPIDController elevator1PID = elevator1Motor.GetPIDController();
    rev::SparkPIDController elevator2PID = elevator2Motor.GetPIDController();
    rev::SparkRelativeEncoder elevator1Encoder = elevator1Motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::SparkRelativeEncoder elevator2Encoder = elevator2Motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

    frc::DutyCycleEncoder shooterAngleEncoder{RobotMap::SHOOTER_ENCODER_ADDRESS};

    Limelight *limelight3;

    double elevatorSpeed = 0.8;
    double alignmentDifference = 0;
    // Should be a constant eventually
    double ampAngle = 22;
    double intakeAngle = 45;
    // Also should be a constant
    double elevator1BottomLimit = 0;
    double elevator1TopLimit = 0;
    double elevator2BottomLimit = 0;
    double elevator2TopLimit = 0;
};