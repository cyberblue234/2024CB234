// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"
#include "Drivetrain.h"

#include "ctre/Phoenix.h"
#include <frc/geometry/Rotation2d.h>
#include <numbers>

// SwerveModule constructor
SwerveModule::SwerveModule(int driveMotorChannel, int swerveMotorChannel, int canCoderChannel, double offsetDegrees)
    : driveMotor(driveMotorChannel, "rio"),
      swerveMotor(swerveMotorChannel, "rio"),
      canCoder(canCoderChannel, "rio")
{
    swerveMotor.ConfigFactoryDefault();
    // Select the canCoder to assign to RemoteSensor0
    swerveMotor.ConfigRemoteFeedbackFilter(canCoder, 0);
    // Select RemoteSensor0 (canCoder) as the selected feedback sensor
    swerveMotor.ConfigSelectedFeedbackSensor(RemoteFeedbackDevice::RemoteFeedbackDevice_RemoteSensor0);
    // Set the sensor phase
    swerveMotor.SetSensorPhase(true);
    swerveMotor.SetInverted(true);
    // Config max voltage to motor
    swerveMotor.ConfigVoltageCompSaturation(11.0);
    swerveMotor.EnableVoltageCompensation(true);

    // Set PID values for angle motor
    swerveMotor.Config_kP(0, kAngleP);
    swerveMotor.Config_kI(0, kAngleI);
    swerveMotor.Config_kD(0, kAngleD);
    swerveMotor.Config_kF(0, kAngleF);

    driveMotor.ConfigFactoryDefault();
    driveMotor.SetSelectedSensorPosition(0);
    driveMotor.ConfigVoltageCompSaturation(11.0);
    driveMotor.EnableVoltageCompensation(true);
    driveMotor.SetNeutralMode(NeutralMode::Brake);

    driveMotor.Config_kP(0, kDriveP);
    driveMotor.Config_kI(0, kDriveI);
    driveMotor.Config_kD(0, kDriveD);
    driveMotor.Config_kF(0, kDriveF);

    canCoder.ConfigFactoryDefault();
    canCoder.ConfigMagnetOffset(offsetDegrees);
    canCoder.SetPosition(0);
    swerveMotor.SetSelectedSensorPosition(0);
}

// Gets the relative rotational position of the module
// Return the relative rotational position of the angle motor in degrees
// GetAbsolutePosition returns 0 - 360 degrees (default)
frc::Rotation2d SwerveModule::GetAngle()
{
    return (frc::Rotation2d(units::angle::degree_t(canCoder.GetAbsolutePosition())));
}

// Set the speed + rotation of the swerve module from a SwerveModuleState object
// param: desiredState - A SwerveModuleState representing the desired new state of the module
void SwerveModule::SetDesiredState(const frc::SwerveModuleState desiredState, double speedAdjustment)
{
    currentAngle = GetAngle(); // 0 - 360 degrees
    // SwerveModuleState contains information about the velocity and angle of a swerve module
    // Optimize to avoid spinning more than 90 degrees, or pi/2 radians
    frc::SwerveModuleState optimizedState = frc::SwerveModuleState::Optimize(desiredState, currentAngle);

    // Find the difference between our current rotational position and our new rotational position
    deltaAngle = optimizedState.angle.operator-(currentAngle);

    // Find how much to turn the module in CANCoder ticks
    deltaCount = ((double)deltaAngle.Degrees() / 360.0) * kCancoderCountsPerRotation;

    // Get the current position of the module in CANCoder ticks
    // Divide by the feedback coefficient to convert from degrees to ticks
    // GetPosition defaults to return degrees.
    currentCount = canCoder.GetPosition() / kCancoderFeedbackCoefficient;

    // The new module position will be the the current ticks plus the change in ticks
    desiredCount = currentCount + deltaCount;
    swerveMotor.Set(TalonFXControlMode::Position, desiredCount);

    // Set the drive motor to the optimized state speed

    percentSpeed = optimizedState.speed / Drivetrain::MAX_SPEED;
    driveMotor.Set(TalonFXControlMode::PercentOutput, percentSpeed * speedAdjustment);
}