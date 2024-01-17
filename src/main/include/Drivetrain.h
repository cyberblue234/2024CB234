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

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>

#include "ctre/Phoenix.h"
#include "AHRS.h"

#include "RobotMap.h"
#include "SwerveModule.h"

// Pathplanner requires class to be a subclass of the frc2::Subsystem 
//  - also gives way for us to use periodic in each subsystem (I think) as well as other benefits
class Drivetrain : frc2::Subsystem
{
public:
    Drivetrain();

    // Main function of the drivetrain, runs all things related to driving
    void DriveControl();
    // Gets all of the joystick values and does calculations, then runs Drive(). Provided bool slows down the speed if true
    void DriveWithJoystick(bool limitSpeed);
    // Sets all of the motors using kinematics calulcations. Uses the provided ChassisSpeeds for calculations
    void Drive(frc::ChassisSpeeds speeds);
    // Updates the odometry. Must be ran every cycle.
    frc::Pose2d UpdateOdometry();
    // Returns the Pose2d of the robot
    frc::Pose2d GetPose() { return odometry.GetPose(); };
    // Resets the Pose2d of the robot
    void ResetPose(frc::Pose2d newPose) { odometry.ResetPosition(gyro.GetRotation2d(),
        {frontLeft.GetModulePosition(), frontRight.GetModulePosition(),
        backLeft.GetModulePosition(), backRight.GetModulePosition()},
        newPose); };
    // Returns the current ChassisSpeeds of the robot
    frc::ChassisSpeeds GetCurrentSpeeds() { return chassisSpeeds; };

    // Sets the motors in an X shape and sets the speed to zero
    void SetAnchorState();
    // Drives at a provided speed and angle
    void DriveAtAngle(double, double);
    // Sets all motors to a speed of zero
    void SetDriveStop();
    // Only allows strafing, speed is considerably reduced
    void DriveSidewaysSlow();
    // Sets all of the motors ramp (the minimun time to accelerate to full throttle)
    void SetDriveOpenLoopRamp(double ramp);
    // Resets all of the motors swerve cancoders
    void ResetCancoders();
    // Resets the gyro, if first time since auton reverses the angle
    void ResetGyroAngle();
    // Resets the gyro for auto
    void ResetGyroForAuto();
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


    static constexpr units::meters_per_second_t MAX_SPEED = 4.084_mps;                // 13.4_fps;
    static constexpr units::radians_per_second_t MAX_ANGULAR_SPEED{std::numbers::pi}; // 1/2 rotation per sec
    
    static constexpr double FL_DRIVE_ADJUSTMENT = 1.0;
    static constexpr double FR_DRIVE_ADJUSTMENT = 1.0;
    static constexpr double BL_DRIVE_ADJUSTMENT = 1.0;
    static constexpr double BR_DRIVE_ADJUSTMENT = 1.0;

    pathplanner::HolonomicPathFollowerConfig holonomicConfig {
            pathplanner::PIDConstants(SwerveModule::kDriveP, SwerveModule::kDriveI, SwerveModule::kDriveD), // Translation PID constants
            pathplanner::PIDConstants(SwerveModule::kAngleP, SwerveModule::kAngleI, SwerveModule::kAngleD), // Rotation PID constants
            MAX_SPEED, // Max module speed, in m/s
            frontLeftLocation.Norm(), // Drive base radius in meters. Distance from robot center to furthest module.
            pathplanner::ReplanningConfig() // Default path replanning config. See the API for the options here
    };

private:

    frc::Translation2d frontLeftLocation{+0.4191_m, +0.4191_m}; 
    frc::Translation2d frontRightLocation{+0.4191_m, -0.4191_m};
    frc::Translation2d backLeftLocation{-0.4191_m, +0.4191_m};
    frc::Translation2d backRightLocation{-0.4191_m, -0.4191_m};
    
    SwerveModule frontLeft{FL_DRIVE_ADDRESS, FL_SWERVE_ADDRESS, FL_CANCODER_ADDRESS, FL_OFFSET_DEGREES};
    SwerveModule frontRight{FR_DRIVE_ADDRESS, FR_SWERVE_ADDRESS, FR_CANCODER_ADDRESS, FR_OFFSET_DEGREES};
    SwerveModule backLeft{BL_DRIVE_ADDRESS, BL_SWERVE_ADDRESS, BL_CANCODER_ADDRESS, BL_OFFSET_DEGREES};
    SwerveModule backRight{BR_DRIVE_ADDRESS, BR_SWERVE_ADDRESS, BR_CANCODER_ADDRESS, BR_OFFSET_DEGREES};

    AHRS gyro{frc::SPI::Port::kMXP};
    double lastGyroPitch;
    double lastGyroRoll;
    bool alignmentOn = false;
    bool gyro_reset_reversed = false;
    frc::ChassisSpeeds chassisSpeeds;

    frc::SwerveDriveKinematics<4> kinematics{
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation};

    frc::SwerveDriveOdometry<4> odometry{kinematics, gyro.GetRotation2d(),
        {frontLeft.GetModulePosition(), frontRight.GetModulePosition(),
        backLeft.GetModulePosition(), backRight.GetModulePosition()},
        frc::Pose2d{0_m, 0_m, 0_rad}};
};

#endif