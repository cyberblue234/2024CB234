#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/length.h>

#include <numbers>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>
#include "Constants.h"

using namespace ctre::phoenix6;
using namespace SwerveModuleConstants;

class SwerveModule
{
public:
    SwerveModule(int driveMotorID, int turnMotorID, int canCoderID, double canCoderMagnetOffset);

    frc::SwerveModuleState GetState();
    frc::SwerveModulePosition GetPosition();
    void SetDesiredState(const frc::SwerveModuleState &state);

    // Returns the velocity of the drive motor
    units::meters_per_second_t GetDriveVelocity() { return units::meters_per_second_t(driveMotor.GetRotorVelocity().GetValueAsDouble() * kWheelRadius * 2 * std::numbers::pi / kDriveGearRatio); };
    // Returns the current being pulled by the drive motor
    double GetDriveCurrent() { return driveMotor.GetMotorVoltage().GetValueAsDouble(); };
    // Returns the current being pulled by the turn motor
    double GetTurnCurrent() { return turnMotor.GetMotorVoltage().GetValueAsDouble(); };
    // Returns a pointer to the drive motor
    const hardware::TalonFX *GetDriveMotor() { return &driveMotor; };
    // Returns a pointer to the turn motor
    const hardware::TalonFX *GetTurnMotor() { return &turnMotor; };
    // Returns the angle of the module [0, 2π]
    units::radian_t GetCanCoderDistance() { return units::radian_t(canCoder.GetAbsolutePosition().GetValueAsDouble() * 2 * std::numbers::pi); };
    // Returns the distance driven by the module
    double GetEncoderDistance() { return driveMotor.GetPosition().GetValueAsDouble() * kWheelRadius * 2 * std::numbers::pi / kDriveGearRatio; };
    // Resets the turn motor's cancoder
    void ResetCanCoder() { canCoder.SetPosition(units::angle::turn_t(0)); }
    // Resets the drive motor's encoder
    void ResetEncoder() { driveMotor.SetPosition(units::angle::turn_t(0)); };

private:
    hardware::TalonFX driveMotor;
    hardware::TalonFX turnMotor;
    hardware::CANcoder canCoder;

    frc::PIDController drivePIDController{kDriveP, kDriveI, kDriveD};
    frc::ProfiledPIDController<units::radians> turnPIDController{kTurnP, kTurnI, kTurnD, {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

    frc::SimpleMotorFeedforward<units::meters> driveFeedforward{kDrive_kS, kDrive_kV};
    frc::SimpleMotorFeedforward<units::radians> turnFeedforward{kTurn_kS, kTurn_kV};
};