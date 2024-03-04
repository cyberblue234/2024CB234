#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "frc/geometry/Pose2d.h"
#include <frc/DigitalInput.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include "rev/CANSparkMax.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>
#include "subsystems/Limelight.h"
#include "Constants.h"
#include <numbers>

using namespace ctre::phoenix6;

class Elevator : frc2::SubsystemBase
{

public:
    Elevator(Limelight *);
    void Periodic() override;

    double CalculateSpeakerAngle();

    void SetElevatorMotorsPosition(double pos) 
    { 
        SetElevator1MotorPosition(pos);
        SetElevator2MotorPosition(pos);
    };
    void SetElevator1MotorPosition(double pos) { SetElevator1Motor(ElevatorPIDCalculate(pos)); };
    void SetElevator2MotorPosition(double pos) { SetElevator2Motor(ElevatorPIDCalculate(pos)); };

    double ElevatorPIDCalculate(double pos) { return elevatorPID.Calculate((double) shooterAngleEncoder.Get(), pos); };
    
    void SetElevatorMotors(double power) 
    { 
        SetElevator1Motor(power); 
        SetElevator2Motor(power); 
    };
    void SetElevator1Motor(double power) { elevator1Motor.Set(power); };
    void SetElevator2Motor(double power) { elevator2Motor.Set(power); };

    double GetShooterAngle() { return shooterAngleEncoder.GetDistance(); };
    double GetShooterAngleRevolutions() { return (double)shooterAngleEncoder.Get(); };
    double GetElevatorSpeed() { return elevatorSpeed; };
    double GetAlignmentDifference() { return alignmentDifference; };
    double GetAmpAngle() { return ampAngle; };
    double GetCloseAngle() { return closeAngle; };
    double GetMidAngle() { return midAngle; };
    double GetStageAngle() { return stageAngle; };
    double GetTrapAngle() { return trapAngle; };
    double GetIntakeAngle() { return intakeAngle; };

    bool GetElevator1TopLimit() { return elevator1TopLimit.Get(); };
    bool GetElevator1BottomLimit() { return elevator1BottomLimit.Get(); };
    bool GetElevator2TopLimit() { return elevator2TopLimit.Get(); };
    bool GetElevator2BottomLimit() { return elevator2BottomLimit.Get(); };

    void ResetShooterEncoder() { shooterAngleEncoder.Reset(); };

    void UpdateTelemetry();

private:
    hardware::TalonFX elevator1Motor{RobotMap::ELEVATOR_MOTOR1_ADDRESS, "rio"};
    hardware::TalonFX elevator2Motor{RobotMap::ELEVATOR_MOTOR2_ADDRESS, "rio"};
    frc::PIDController elevatorPID{ElevatorConstants::kElevatorP, ElevatorConstants::kElevatorI, ElevatorConstants::kElevatorD, 20_ms};

    frc::DutyCycleEncoder shooterAngleEncoder{RobotMap::SHOOTER_ENCODER_ADDRESS};

    frc::DigitalInput elevator1TopLimit{RobotMap::ELEVATOR1_TOP_LIMIT_SWITCH};
    frc::DigitalInput elevator1BottomLimit{RobotMap::ELEVATOR1_BOTTOM_LIMIT_SWITCH};
    frc::DigitalInput elevator2TopLimit{RobotMap::ELEVATOR2_TOP_LIMIT_SWITCH};
    frc::DigitalInput elevator2BottomLimit{RobotMap::ELEVATOR2_BOTTOM_LIMIT_SWITCH};

    Limelight *limelight3;

    double elevatorSpeed = 1.0;
    double alignmentDifference = 0;
    // Should be a constant eventually
    double ampAngle = 22;
    double closeAngle = 50;
    double midAngle = 30;
    double stageAngle = 30;
    double trapAngle = 30;
    double intakeAngle = 45;
};