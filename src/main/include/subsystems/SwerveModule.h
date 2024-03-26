#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/geometry/Rotation2d.h>

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

class SwerveModule
{
public:
    SwerveModule(int driveMotorChannel, int swerveMotorChannel, int canCoderChannel, double offsetDegrees);

    // Gets the relative rotational position of the module
    // Return the relative rotational position of the angle motor in degrees
    // GetAbsolutePosition returns 0 - 360 degrees (default)
    frc::Rotation2d GetAngle() { return (frc::Rotation2d(units::angle::degree_t(GetSwervePosition()))); }

    void SetDesiredState(frc::SwerveModuleState desiredState, double speedAdjustment);
    // Returns the desired count for the swerve encoder
    double GetDesiredCount() { return desiredCount; }
    // Returns the current count of the swerve motor encoder
    double GetCurrentCount() { return currentCount; }
    // Returns the amount of encoder ticks needed to turn to the desired angle
    double GetDeltaCount() { return deltaCount; }
    // Returns the current angle of the robot
    double GetCurrentAngle() { return (double)currentAngle.Degrees(); }
    // Returns the amount of degrees needed to turn to the desired angle
    double GetDeltaAngle() { return (double)deltaAngle.Degrees(); }
    // Returns the speed of the motor
    double GetPercentSpeed() { return percentSpeed; }
    // Returns the abosolute position of the swerve motor
    double GetSwervePosition() { return canCoder.GetAbsolutePosition().GetValueAsDouble() * 360.0; }
    // Returns the encoder position of the drive motor
    double GetDriveEncoder() { return driveMotor.GetPosition().GetValueAsDouble(); };
    // Returns the current SwerveModulePosition
    frc::SwerveModulePosition GetModulePosition() 
    { 
        return frc::SwerveModulePosition{units::length::meter_t(GetDriveEncoder() / SwerveModuleConstants::DRIVE_GEAR_RATIO * SwerveModuleConstants::WHEEL_CIRCUMFERENCE), GetAngle()}; 
    };
    // Returns the RPM of the drive motor
    double GetDriveRPM() { return driveMotor.GetRotorVelocity().GetValueAsDouble() * 600.0 / 2048.0; };
    // Returns the current being pulled by the drive motor
    double GetDriveCurrent() { return driveMotor.GetMotorVoltage().GetValueAsDouble(); };
    // Returns the current being pulled by the swerve motor
    double GetSwerveCurrent() { return swerveMotor.GetMotorVoltage().GetValueAsDouble(); };
    // Sets the drive motor to a provided speed
    void SetDriveMotor(double speed) { driveMotor.Set(speed); };
    // Sets the swerve motor to a provided speed
    void SetSwerveMotor(double speed) { swerveMotor.Set(speed); };
    // Returns a pointer to the drive motor
    const hardware::TalonFX *GetDriveMotor() { return &driveMotor; };
    // Returns a pointer to the swerve motor
    const hardware::TalonFX *GetSwerveMotor() { return &swerveMotor; };
    // Resets the swerve motor's cancoder
    void ResetCanCoder() { canCoder.SetPosition(units::angle::turn_t(0)); }
    // Resets the drive motor's encoder
    void ResetEncoder() { driveMotor.SetPosition(units::angle::turn_t(0)); };

private:
    double desiredCount;
    double currentCount;
    double deltaCount;
    double percentSpeed;

    frc::Rotation2d currentAngle;
    frc::Rotation2d deltaAngle;

    hardware::TalonFX driveMotor;
    hardware::TalonFX swerveMotor;
    hardware::CANcoder canCoder;

    controls::PositionVoltage swervePositionOut{0_tr, 0_tps, true, 0_V, 0, false};
    // controls::DutyCycleOut driveMotorOut(0);
};