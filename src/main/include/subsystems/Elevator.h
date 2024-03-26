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
#include "subsystems/Drivetrain.h"
#include "Constants.h"
#include <numbers>

using namespace ctre::phoenix6;

class Elevator : frc2::SubsystemBase
{

public:
    Elevator(Limelight *, Drivetrain *);
    void Periodic() override;

    double CalculateSpeakerAngle(bool useLimelight);

    void ElevatorControl(double angle);

    void ToIntakeAngle() { ElevatorControl(intakeAngle); };
    void ToSubwooferAngle() { ElevatorControl(closeAngle); };
    void ToCalculatedAngle(bool useLimelight) { ElevatorControl(CalculateSpeakerAngle(useLimelight)); };

    void SetElevatorMotorsPosition(double pos) 
    { 
        SetElevator1MotorPosition(pos);
        SetElevator2MotorPosition(pos);
    };
    void SetElevator1MotorPosition(double pos) { SetElevator1Motor(ElevatorPIDCalculate(pos)); };
    void SetElevator2MotorPosition(double pos) { SetElevator2Motor(ElevatorPIDCalculate(pos)); };

    double ElevatorPIDCalculate(double pos) { return elevatorPID.Calculate(GetShooterAngle(), pos); };

    bool AtSetpoint() { return elevatorPID.AtSetpoint(); };

    void SetElevatorMotorsWithLimits(double power);
    void SetElevatorMotors(double power) 
    { 
        SetElevator1Motor(power); 
        SetElevator2Motor(power); 
    };
    void SetElevator1Motor(double power) { elevator1Motor.Set(power); };
    void SetElevator2Motor(double power) { elevator2Motor.Set(power); };

    double GetShooterAngle() { return shooterAngleEncoder.GetDistance() - ElevatorConstants::SHOOTER_ANGLE_OFFSET; };
    double GetShooterAngleRevolutions() { return (double)shooterAngleEncoder.Get() - (ElevatorConstants::SHOOTER_ANGLE_OFFSET / 360); };
    double GetElevatorSpeed() { return elevatorSpeed; };
    double GetAlignmentDifference() { return alignmentDifference; };
    double GetAmpAngle() { return ampAngle; };
    double GetCloseAngle() { return closeAngle; };
    double GetMidAngle() { return midAngle; };
    double GetStageAngle() { return stageAngle; };
    double GetTrapAngle() { return trapAngle; };
    double GetIntakeAngle() { return intakeAngle; };

    bool GetElevator1BottomLimit() { return elevator1BottomLimit.Get(); };
    bool GetElevator2BottomLimit() { return elevator2BottomLimit.Get(); };

    double GetElevator1MotorRPM() { return elevator1Motor.GetVelocity().GetValueAsDouble(); };
    double GetElevator2MotorRPM() { return elevator2Motor.GetVelocity().GetValueAsDouble(); };

    void ResetShooterEncoder() { shooterAngleEncoder.Reset(); };

    double GetElevator1Encoder() { return elevator1Motor.GetPosition().GetValueAsDouble(); };
    double GetElevator2Encoder() { return elevator2Motor.GetPosition().GetValueAsDouble(); };
    void ResetElevator1Encoder() { elevator1Motor.SetPosition(units::angle::turn_t(0.0)); };
    void ResetElevator2Encoder() { elevator2Motor.SetPosition(units::angle::turn_t(0.0)); };

    double GetHardEncoderLimit() { return hardEncoderLimit; };

    const hardware::TalonFX *GetElevator1Motor() { return &elevator1Motor; };
    const hardware::TalonFX *GetElevator2Motor() { return &elevator2Motor; };
    
    void UpdateTelemetry();

private:
    hardware::TalonFX elevator1Motor{RobotMap::ELEVATOR_MOTOR1_ADDRESS, "rio"};
    hardware::TalonFX elevator2Motor{RobotMap::ELEVATOR_MOTOR2_ADDRESS, "rio"};
    frc::PIDController elevatorPID{ElevatorConstants::kElevatorP, ElevatorConstants::kElevatorI, ElevatorConstants::kElevatorD, 20_ms};

    frc::DutyCycleEncoder shooterAngleEncoder{RobotMap::SHOOTER_ENCODER_ADDRESS};

    frc::DigitalInput elevator1BottomLimit{RobotMap::ELEVATOR1_BOTTOM_LIMIT_SWITCH};
    frc::DigitalInput elevator2BottomLimit{RobotMap::ELEVATOR2_BOTTOM_LIMIT_SWITCH};

    Limelight *limelight3;
    Drivetrain *swerve;

    double elevatorSpeed = 1.0;
    double alignmentDifference = 0;
    // Should be a constant eventually
    double ampAngle = -54;
    double closeAngle = -52;
    double midAngle = -44;
    double stageAngle = -30;
    double trapAngle = -30;
    double intakeAngle = -44;

    double hardEncoderLimit = 200;
};