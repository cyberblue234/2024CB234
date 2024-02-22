#pragma once

#include "Constants.h"
#include "frc/geometry/Pose2d.h"
#include "rev/CANSparkMax.h"
#include <frc2/command/SubsystemBase.h>

class Elevator : frc2::SubsystemBase 
{

public:
    Elevator();
    void Periodic() override;

    void SetElevatorMotorsPosition(double pos) { elevatorPID.SetReference(pos, rev::CANSparkLowLevel::ControlType::kPosition); };
    void SetElevatorMotors(double power) { elevatorMotor1.Set(power); };

    double CalculateSpeakerAngle();

private:
    rev::CANSparkMax elevatorMotor1{RobotMap::ELEVATOR_MOTOR1_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax elevatorMotor2{RobotMap::ELEVATOR_MOTOR2_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkPIDController elevatorPID = elevatorMotor1.GetPIDController();
};