// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// #include <frc/Encoder.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/length.h>
#include <numbers>

#include "ctre/Phoenix.h"

class SwerveModule
{
public:
    SwerveModule(int driveMotorChannel, int swerveMotorChannel, int canCoderChannel, double offsetDegrees);

    // Gets the relative rotational position of the module
    // Return the relative rotational position of the angle motor in degrees
    // GetAbsolutePosition returns 0 - 360 degrees (default)
    frc::Rotation2d GetAngle() { return (frc::Rotation2d(units::angle::degree_t(canCoder.GetAbsolutePosition()))); }
    
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
    double GetSwervePosition() { return canCoder.GetAbsolutePosition(); }
    // Returns the encoder position of the drive motor
    double GetDriveEncoder() { return driveMotor.GetSelectedSensorPosition(); };
    // Returns the current SwerveModulePosition
    frc::SwerveModulePosition GetModulePosition() { return frc::SwerveModulePosition{units::length::meter_t(GetDriveEncoder() * ENCODER_METERS_PER_COUNT), GetAngle()}; };
    // Returns the RPM of the drive motor
    double GetDriveRPM() { return driveMotor.GetSelectedSensorVelocity() * 600.0 / 2048.0; };
    // Returns the current being pulled by the drive motor
    double GetDriveCurrent() { return driveMotor.GetOutputCurrent(); };
    // Returns the current being pulled by the swerve motor
    double GetSwerveCurrent() { return swerveMotor.GetOutputCurrent(); };
    // Sets the drive motor to a provided speed
    void SetDriveMotor(double speed) { driveMotor.Set(TalonFXControlMode::PercentOutput, speed); };
    // Sets the swerve motor to a provided speed
    void SetSwerveMotor(double speed) { swerveMotor.Set(TalonFXControlMode::PercentOutput, speed); };
    // Sets the ramp for the drive motor (the minimun time to accelerate to full throttle)
    void SetDriveOpenLoopRamp(double ramp) { driveMotor.ConfigOpenloopRamp(ramp); };
    // Resets the swerve motor's cancoder
    void ResetCancoder() { canCoder.SetPosition(0); }
    // Resets the drive motor's encoder
    void ResetEncoder() { driveMotor.SetSelectedSensorPosition(0); };

    static constexpr double ENCODER_INCHES_PER_COUNT = 0.00090689;
    static constexpr double ENCODER_METERS_PER_COUNT = ENCODER_INCHES_PER_COUNT / 39.37;
        // 6.54 : 1

private:
    const double kDriveP = 15.0;
    const double kDriveI = 0.01;
    const double kDriveD = 0.1;
    const double kDriveF = 0.2;

    const double kAngleP = 0.30;
    const double kAngleI = 0.002;
    const double kAngleD = 0.0;
    const double kAngleF = 0.0;

    const double kCancoderCountsPerRotation = 4096;
    const double kCancoderFeedbackCoefficient = 0.087890625;

    double desiredCount;
    double currentCount;
    double deltaCount;
    double percentSpeed;
    frc::Rotation2d currentAngle;
    frc::Rotation2d deltaAngle;

    WPI_TalonFX driveMotor;
    WPI_TalonFX swerveMotor;
    WPI_CANCoder canCoder;
};