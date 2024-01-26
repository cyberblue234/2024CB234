#ifndef _DRIVETRAIN_H
#define _DRIVETRAIN_H

#include <numbers>

#include <frc/XboxController.h>
#include <frc/DriverStation.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include <frc2/command/SubsystemBase.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

#include "ctre/Phoenix.h"
#include "AHRS.h"

#include "subsystems/SwerveModule.h"
#include "Constants.h"

class Drivetrain : frc2::SubsystemBase
{
public:
    Drivetrain();

    void Periodic() override;

    // Main function of the drivetrain, runs all things related to driving
    void DriveControl();
    // Gets all of the joystick values and does calculations, then runs Drive(). Provided bool slows down the speed if true
    void DriveWithJoystick(bool limitSpeed);
    // Sets all of the motors using kinematics calulcations. Uses the provided ChassisSpeeds for calculations
    void Drive(const frc::ChassisSpeeds& speeds);
    // Returns the Pose2d of the robot
    const frc::Pose2d& GetPose() const { return odometry.GetPose(); };
    // Resets the Pose2d of the robot
    void ResetPose(const frc::Pose2d& pose) { odometry.ResetPosition(gyro.GetRotation2d(), { frontLeft.GetModulePosition(), frontRight.GetModulePosition(), backLeft.GetModulePosition(), backRight.GetModulePosition() }, pose ); };
    // Returns the current ChassisSpeeds of the robot
    frc::ChassisSpeeds GetCurrentSpeeds() { return chassisSpeeds; };

    // Sets all of the motors ramp (the minimun time to accelerate to full throttle)
    void SetDriveOpenLoopRamp(double ramp);
    // Resets all of the motors swerve cancoders
    void ResetCancoders();
    // Resets the gyro, if first time since auton reverses the angle
    void ResetGyroAngle();
    // Resets the gyro for auto
    void ResetGyroForAuto();
    //Gets the gyro as a rotation2d
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
    // Returns the front left drive motor's RPM
    double GetDriveRPM() { return abs(frontLeft.GetDriveRPM()); };
    // Returns the front left drive motor's RPM
    double GetLeftDriveRPM() { return abs(frontLeft.GetDriveRPM()); };
    // Returns the front right drive motor's RPM
    double GetRightDriveRPM() { return abs(frontRight.GetDriveRPM()); };
    // Returns the back left drive motor's RPM
    double GetBackLeftDriveRPM() { return abs(backLeft.GetDriveRPM()); };
    // Returns the back right drive motor's RPM
    double GetBackRightDriveRPM() { return abs(backRight.GetDriveRPM()); };
    // Returns the front left swerve cancoder's value
    double GetFrontLeftAngle() { return frontLeft.GetCurrentAngle(); };
    // Returns the front right swerve cancoder's value
    double GetFrontRightAngle() { return frontRight.GetCurrentAngle(); };
    // Returns the back left swerve cancoder's value
    double GetBackLeftAngle() { return backLeft.GetCurrentAngle(); };
    // Returns the back right swerve cancoder's value
    double GetBackRightAngle() { return backRight.GetCurrentAngle(); };
    // Returns the average of the front left and back right distance
    double GetDriveDistance();


    // Returns the current being pulled by the front left drive motor
    double GetFLDriveCurrent() { return frontLeft.GetDriveCurrent(); };
    // Returns the current being pulled by the front right drive motor
    double GetFRDriveCurrent() { return frontRight.GetDriveCurrent(); };
    // Returns the current being pulled by the back left drive motor
    double GetBLDriveCurrent() { return backLeft.GetDriveCurrent(); };
    // Returns the current being pulled by the back right drive motor
    double GetBRDriveCurrent() { return backRight.GetDriveCurrent(); };
    // Returns the current being pulled by the front left swerve motor
    double GetFLSwerveCurrent() { return frontLeft.GetSwerveCurrent(); };
    // Returns the current being pulled by the front right swerve motor
    double GetFRSwerveCurrent() { return frontRight.GetSwerveCurrent(); };
    // Returns the current being pulled by the back left swerve motor
    double GetBLSwerveCurrent() { return backLeft.GetSwerveCurrent(); };
    // Returns the current being pulled by the back right swerve motor
    double GetBRSwerveCurrent() { return backRight.GetSwerveCurrent(); };

    // Resets all drive motor encoders
    void ResetDriveEncoders();
    // Sets all motors to a speed of zero
    void AlignSwerveDrive();

private:
    SwerveModule frontLeft;
    SwerveModule frontRight;
    SwerveModule backLeft;
    SwerveModule backRight;

    AHRS gyro;
    double lastGyroPitch;
    double lastGyroRoll;
    bool alignmentOn = false;
    bool gyro_reset_reversed = false;
    frc::ChassisSpeeds chassisSpeeds;

    frc::SwerveDriveKinematics<4> kinematics
    {
        DrivetrainConstants::frontLeftLocation, 
        DrivetrainConstants::frontRightLocation, 
        DrivetrainConstants::backLeftLocation, 
        DrivetrainConstants::backRightLocation
    };

    frc::SwerveDriveOdometry<4> odometry;
};

#endif