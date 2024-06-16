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
#include <functional>

using namespace ctre::phoenix6;

class Elevator : frc2::SubsystemBase
{

public:
    Elevator();
    void Periodic() override;

    void ElevatorControl(double value);

    void SetElevator1Motor(double power) { elevator1Motor.Set(power); };
    void SetElevator2Motor(double power) { elevator2Motor.Set(power); };

    void StopMotors() { SetElevator1Motor(0.0); SetElevator2Motor(0.0); };
    
    double GetElevatorSpeed() { return elevatorSpeed; };

    bool GetElevator1BottomLimit() { return !elevator1BottomLimit.Get(); };
    bool GetElevator2BottomLimit() { return !elevator2BottomLimit.Get(); };

    double GetElevator1MotorRPM() { return elevator1Motor.GetVelocity().GetValueAsDouble(); };
    double GetElevator2MotorRPM() { return elevator2Motor.GetVelocity().GetValueAsDouble(); };

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

    frc::DigitalInput elevator1BottomLimit{RobotMap::ELEVATOR1_BOTTOM_LIMIT_SWITCH};
    frc::DigitalInput elevator2BottomLimit{RobotMap::ELEVATOR2_BOTTOM_LIMIT_SWITCH};

    double elevatorSpeed = 0.75;

    double hardEncoderLimit = 150;
};