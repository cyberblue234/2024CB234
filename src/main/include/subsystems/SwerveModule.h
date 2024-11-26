#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>

#include <frc/geometry/Rotation2d.h>

#include <frc/controller/SimpleMotorFeedforward.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/sendable/SendableRegistry.h>

#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <units/voltage.h>
#include <units/current.h>
#include <units/temperature.h>
#include <units/time.h>

#include <numbers>
#include <string>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configurator.hpp>

#include <ctre/phoenix6/sim/CANcoderSimState.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <frc/simulation/DCMotorSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/AnalogGyroSim.h>

#include "Constants.h"

using namespace ctre::phoenix6;
using namespace SwerveModuleConstants;

class SwerveModule
{
public:
    SwerveModule(std::string name, int driveMotorID, int turnMotorID, int canCoderID, double canCoderMagnetOffset);

    frc::SwerveModuleState GetState() { return {GetDriveVelocity(), GetAngle()}; };
    frc::SwerveModulePosition GetPosition() { return {GetDistance(), GetAngle()}; };
    void SetDesiredState(const frc::SwerveModuleState &state);

    void UpdateTelemetry();
    void SimMode();

    // Returns the distance driven by the module
    units::meter_t GetDistance() { return units::meter_t(driveMotor.GetPosition().GetValueAsDouble() * kDriveDistanceRatio); };
    // Returns the velocity of the drive motor
    units::meters_per_second_t GetDriveVelocity() { return units::meters_per_second_t(driveMotor.GetRotorVelocity().GetValueAsDouble() * kDriveDistanceRatio); };
    // Returns the angle of the module [0, 2π), [0, 360)
    frc::Rotation2d GetAngle() { return frc::Rotation2d{units::radian_t(canCoder.GetAbsolutePosition().GetValueAsDouble() * kTurnDistanceRatio)}; };
    
    // Returns the applied voltage of the drive motor
    units::voltage::volt_t GetDriveVoltage() { return driveMotor.GetMotorVoltage().GetValue(); };
    // Returns the applied voltage of the turn motor
    units::voltage::volt_t GetTurnVoltage() { return turnMotor.GetMotorVoltage().GetValue(); };
    // Returns the torque current of the drive motor
    units::current::ampere_t GetDriveTorqueCurrent() { return driveMotor.GetTorqueCurrent().GetValue(); };
    // Returns the torque current of the turn motor
    units::current::ampere_t GetTurnTorqueCurrent() { return turnMotor.GetTorqueCurrent().GetValue(); };
    // Returns the stator current of the drive motor
    units::current::ampere_t GetDriveStatorCurrent() { return driveMotor.GetStatorCurrent().GetValue(); };
    // Returns the stator current of the turn motor
    units::current::ampere_t GetTurnStatorCurrent() { return turnMotor.GetStatorCurrent().GetValue(); };
    // Returns the supply current of the drive motor
    units::current::ampere_t GetDriveSupplyCurrent() { return driveMotor.GetSupplyCurrent().GetValue(); };
    // Returns the supply current of the turn motor
    units::current::ampere_t GetTurnSupplyCurrent() { return turnMotor.GetSupplyCurrent().GetValue(); };
    // Returns the temperature of the drive motor
    units::temperature::celsius_t GetDriveTemp() { driveMotor.GetDeviceTemp().GetValue(); };
    // Returns the temperature of the turn motor
    units::temperature::celsius_t GetDriveTemp() { turnMotor.GetDeviceTemp().GetValue(); };
    // Returns the temperature of the drive motor processor
    units::temperature::celsius_t GetDriveProcessorTemp() { driveMotor.GetProcessorTemp().GetValue(); };
    // Returns the temperature of the turn motor processor
    units::temperature::celsius_t GetDriveProcessorTemp() { turnMotor.GetProcessorTemp().GetValue(); };

    // Returns a pointer to the drive motor
    hardware::TalonFX *GetDriveMotor() { return &driveMotor; };
    // Returns a pointer to the turn motor
    hardware::TalonFX *GetTurnMotor() { return &turnMotor; };
    // Returns a pointer to the CAN coder
    hardware::CANcoder *GetCANcoder() { return &canCoder; };
    
    // Sets the drive motor's encoder
    void SetEncoder(double value) { driveMotor.SetPosition(units::angle::turn_t(value)); };
    // Sets the turn motor's cancoder
    void SetCanCoder(double value) { canCoder.SetPosition(units::angle::turn_t(value)); }

private:
    std::string name;

    hardware::TalonFX driveMotor;
    hardware::TalonFX turnMotor;
    hardware::CANcoder canCoder;

    controls::PositionVoltage turnPositionOut{0_tr, 0_tps, false, 0_V, 0, false};
    controls::VelocityVoltage driveVelocityOut{0, 0, false};

    frc::SimpleMotorFeedforward<units::meters> driveFeedforward{kDrive_kS, kDrive_kV};
    frc::SimpleMotorFeedforward<units::radians> turnFeedforward{kTurn_kS, kTurn_kV};

    frc::sim::DCMotorSim driveMotorSimModel{frc::DCMotor::KrakenX60(1), kDriveGearRatio, 500_kg_sq_m};
    frc::sim::DCMotorSim turnMotorSimModel{frc::DCMotor::KrakenX60(1), kTurnGearRatio, 500_kg_sq_m};
};