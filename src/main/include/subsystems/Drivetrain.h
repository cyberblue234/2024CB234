#pragma once

#ifndef _DRIVETRAIN_H
#define _DRIVETRAIN_H

#include <numbers>
#include <vector>

#include <frc/XboxController.h>
#include <frc/DriverStation.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <frc/controller/PIDController.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include <frc2/command/SubsystemBase.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

#include "AHRS.h"

#include "subsystems/SwerveModule.h"
#include "subsystems/Limelight.h"
#include "Constants.h"
#include "Controls.h"

extern frc::PIDController rotationController;

class Drivetrain : frc2::SubsystemBase
{
public:
    Drivetrain(Limelight *);

    void Periodic() override;

    void DriveControls();

    // Gets all of the joystick values and does calculations, then runs Drive(). Provided bool slows down the speed if true
    void DriveWithInput(double, double, double, bool);
    // Sets all of the motors using kinematics calulcations. Uses the provided ChassisSpeeds for calculations
    void Drive(const frc::ChassisSpeeds &);
    // Updates the odometry. Must be ran every cycle.
    frc::Pose2d UpdateOdometry();
    // Adds Vision data to odometry and returns the newest pose estimation. Also sets the pipeline ID to Apriltags
    frc::Pose2d UpdateOdometryWithVision(bool);
    // Returns the Pose2d of the robot
    frc::Pose2d GetPose() { return odometry.GetEstimatedPosition(); };
    // Resets the Pose2d of the robot
    void ResetPose(frc::Pose2d pose) { odometry.ResetPosition(gyro.GetRotation2d(), {frontLeft.GetModulePosition(), frontRight.GetModulePosition(), backLeft.GetModulePosition(), backRight.GetModulePosition()}, pose); };
    // Returns the current ChassisSpeeds of the robot
    frc::ChassisSpeeds GetCurrentSpeeds() { return chassisSpeeds; };
    // Returns a the input cubed
    double RotationControl(double);
    // Returns the output to align
    double RotationControl();
    // Only rotates bot to april tag
    void AlignToSpeaker();
    // Turns all wheels to an X orientation, preventing most movement
    void SetAnchorState();

    void UpdateTelemetry();

    // Sets the bool that determines if the robot drives field or robot relative
    void SetFieldRelative(bool isFieldRelative) { fieldRelative = isFieldRelative; };
    // Resets all of the motors swerve cancoders
    void ResetCancoders();
    // Resets the gyro
    void ResetGyroAngle();
    // Gets the gyro as a rotation2d
    frc::Rotation2d GetGyro2d() { return gyro.GetRotation2d(); };
    // Returns the gyro's yaw
    double GetGyroAngle() { return gyro.GetYaw(); };
    // Returns the gyro's pitch
    double GetGyroPitch() { return gyro.GetRoll() - lastGyroRoll; };
    // Returns the gyro's roll
    double GetGyroRoll() { return gyro.GetPitch() - lastGyroPitch; };
    // Resets the gyro's pitch
    void ResetGyroPitch() { lastGyroRoll = gyro.GetRoll(); };
    // Resets the gyro's roll
    void ResetGyroRoll() { lastGyroPitch = gyro.GetPitch(); };

    // Returns a pointer to the front left module
    SwerveModule *GetFrontLeftModule() { return &frontLeft; };
    // Returns a pointer to the front right module
    SwerveModule *GetFrontRightModule() { return &frontRight; };
    // Returns a pointer to the back left module
    SwerveModule *GetBackLeftModule() { return &backLeft; };
    // Returns a pointer to the back right module
    SwerveModule *GetBackRightModule() { return &backRight; };
    // Returns a pointer to the Field2d object
    frc::Field2d *GetField() { return &field; };

    // Resets all drive motor encoders
    void ResetDriveEncoders();
    // Sets all motors to a speed of zero
    void AlignSwerveDrive();
    // Returns alignmentOn bool
    bool IsAlignmentOn() { return alignmentOn; };
    // Returns true if the robot is angled correctly within a setpoint (5.0 by default)
    bool AtSetpoint() { return rotationController.AtSetpoint(); };

private:
    SwerveModule frontLeft;
    SwerveModule frontRight;
    SwerveModule backLeft;
    SwerveModule backRight;

    Limelight *limelight3;

    AHRS gyro;
    double lastGyroPitch = 0;
    double lastGyroRoll = 0;
    bool alignmentOn = false;
    bool fieldRelative = true;

    frc::ChassisSpeeds chassisSpeeds;

    frc::SwerveDriveKinematics<4> kinematics{
        DrivetrainConstants::frontLeftLocation,
        DrivetrainConstants::frontRightLocation,
        DrivetrainConstants::backLeftLocation,
        DrivetrainConstants::backRightLocation};

    frc::SwerveDrivePoseEstimator<4> odometry;

    int cycle = 0;

    frc::PIDController rotationController{DrivetrainConstants::kRotationP, DrivetrainConstants::kRotationI, DrivetrainConstants::kRotationD, 20_ms};

    frc::Field2d field;
};

#endif