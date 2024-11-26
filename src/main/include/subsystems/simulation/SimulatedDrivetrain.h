#pragma once

#include <numbers>
#include <vector>

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

#include "AHRS.h"

#include "subsystems/simulation/SimulatedSwerveModule.h"
#include "Constants.h"

#include <wpi/sendable/SendableRegistry.h>

using namespace DrivetrainConstants;

class SimulatedDrivetrain
{
public:
    SimulatedDrivetrain();
    void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, units::second_t period);
    void UpdateOdometry();
    void UpdateTelemetry();

    // Returns a pointer to the front left module
    SimulatedSwerveModule *GetFrontLeftModule() { return &frontLeft; };
    // Returns a pointer to the front right module
    SimulatedSwerveModule *GetFrontRightModule() { return &frontRight; };
    // Returns a pointer to the back left module
    SimulatedSwerveModule *GetBackLeftModule() { return &backLeft; };
    // Returns a pointer to the back right module
    SimulatedSwerveModule *GetBackRightModule() { return &backRight; };
private:
    SimulatedSwerveModule frontLeft{RobotMap::kFrontLeftDriveID, RobotMap::kFrontLeftTurnID, RobotMap::kFrontLeftCanCoderID, kFrontLeftMagnetOffset};
    SimulatedSwerveModule frontRight{RobotMap::kFrontRightDriveID, RobotMap::kFrontRightTurnID, RobotMap::kFrontRightCanCoderID, kFrontRightMagnetOffset};
    SimulatedSwerveModule backLeft{RobotMap::kBackLeftDriveID, RobotMap::kBackLeftTurnID, RobotMap::kBackLeftCanCoderID, kBackLeftMagnetOffset};
    SimulatedSwerveModule backRight{RobotMap::kBackRightDriveID, RobotMap::kBackRightTurnID, RobotMap::kBackRightCanCoderID, kBackRightMagnetOffset};

    AHRS gyro{frc::SPI::Port::kMXP};
    

    frc::SwerveDriveKinematics<4> kinematics{
        kFrontLeftLocation,
        kFrontRightLocation,
        kBackLeftLocation,
        kBackRightLocation};

    frc::SwerveDrivePoseEstimator<4> odometry{
        kinematics,
        gyro.GetRotation2d(),
        {frontLeft.GetPosition(), frontRight.GetPosition(),
         backLeft.GetPosition(), backRight.GetPosition()},
        frc::Pose2d()};
};