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

#include <ctre/phoenix6/sim/CANcoderSimState.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <frc/simulation/DCMotorSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/AnalogGyroSim.h>


using namespace ctre::phoenix6;
using namespace SwerveModuleConstants;

class SimulatedSwerveModule
{
public:
    SimulatedSwerveModule(int driveMotorID, int turnMotorID, int canCoderID, double canCoderMagnetOffset);

    frc::SwerveModuleState GetState() { return {units::meters_per_second_t{GetDriveVelocity()}, units::radian_t{GetCanCoderDistance()}}; };
    frc::SwerveModulePosition GetPosition() { return {units::meter_t{GetEncoderDistance()}, units::radian_t{GetCanCoderDistance()}}; };
    void SetDesiredState(const frc::SwerveModuleState &state);

    // Returns the velocity of the drive motor
    units::meters_per_second_t GetDriveVelocity() { return units::meters_per_second_t(driveMotor.GetRotorVelocity().GetValueAsDouble() * kDriveDistanceRatio); };
    // Returns the current being pulled by the drive motor
    double GetDriveCurrent() { return driveMotor.GetMotorVoltage().GetValueAsDouble(); };
    // Returns the current being pulled by the turn motor
    double GetTurnCurrent() { return turnMotor.GetMotorVoltage().GetValueAsDouble(); };
    // Returns a pointer to the drive motor
    hardware::TalonFX *GetDriveMotor() { return &driveMotor; };
    // Returns a pointer to the turn motor
    hardware::TalonFX *GetTurnMotor() { return &turnMotor; };
    // Returns a pointer to the CAN coder
    hardware::CANcoder *GetCANcoder() { return &canCoder; };
    // Returns the angle of the module [0, 2π]
    units::radian_t GetCanCoderDistance() { return units::radian_t(canCoder.GetAbsolutePosition().GetValueAsDouble() * kTurnDistanceRatio); };
    // Returns the distance driven by the module
    double GetEncoderDistance() { return driveMotor.GetPosition().GetValueAsDouble() * kDriveDistanceRatio; };
    // Resets the turn motor's cancoder
    void ResetCanCoder(double value) { canCoder.SetPosition(units::angle::turn_t(value)); }
    // Resets the drive motor's encoder
    void SetEncoder(double value) { driveMotor.SetPosition(units::angle::turn_t(value)); };

private:
    hardware::TalonFX driveMotor;
    hardware::TalonFX turnMotor;
    hardware::CANcoder canCoder;

    frc::PIDController drivePIDController{kDriveP, kDriveI, kDriveD};
    frc::ProfiledPIDController<units::radians> turnPIDController{kTurnP, kTurnI, kTurnD, {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

    frc::SimpleMotorFeedforward<units::meters> driveFeedforward{kDrive_kS, kDrive_kV};
    frc::SimpleMotorFeedforward<units::radians> turnFeedforward{kTurn_kS, kTurn_kV};

    frc::sim::DCMotorSim driveMotorSimModel{frc::DCMotor::KrakenX60(1), kDriveGearRatio, 0.001_kg_sq_m};
    frc::sim::DCMotorSim turnMotorSimModel{frc::DCMotor::KrakenX60(1), 1, 0.001_kg_sq_m};
};