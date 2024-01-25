#pragma once

#include <frc/geometry/Translation2d.h>
#include <units/velocity.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/PowerDistribution.h>

static frc::PowerDistribution pdp{1, frc::PowerDistribution::ModuleType::kRev};
static frc::XboxController gamePad{0};
static frc::Joystick controls(1);

namespace RobotMap
{
    constexpr int FL_DRIVE_ADDRESS = 8;
    constexpr int FL_SWERVE_ADDRESS = 9;
    constexpr int FL_CANCODER_ADDRESS = 21;

    constexpr int FR_DRIVE_ADDRESS = 2;
    constexpr int FR_SWERVE_ADDRESS = 3;
    constexpr int FR_CANCODER_ADDRESS = 22;

    constexpr int BL_DRIVE_ADDRESS = 7;
    constexpr int BL_SWERVE_ADDRESS = 6;
    constexpr int BL_CANCODER_ADDRESS = 23;

    constexpr int BR_DRIVE_ADDRESS = 4;
    constexpr int BR_SWERVE_ADDRESS = 5;
    constexpr int BR_CANCODER_ADDRESS = 24;


    constexpr int ARM_MOTOR_ADDRESS = 99;
    constexpr int ROTATOR_MOTOR_ADDRESS = 12;
    constexpr int GRIPPER_MOTOR_ADDRESS = 13;

    constexpr int ARM_PCM_ADDRESS = 0;
    constexpr int LED_PCM_ADDRESS = 1;

    // roboRIO Input Channels

    constexpr int ARM_ENCODER_CHANNEL = 0;
    constexpr int ROTATOR_ENCODER_CHANNEL = 1;
    constexpr int GRIPPER_ENCODER_CHANNEL = 2;
    constexpr int PRESSURE_SENSOR_CHANNEL = 3;
    constexpr int ROTATOR_LIMIT_SWITCH_CHANNEL = 9;
}

namespace SwerveModuleConstants 
{
    inline constexpr double ENCODER_INCHES_PER_COUNT = 0.00090689;
    inline constexpr double ENCODER_METERS_PER_COUNT = ENCODER_INCHES_PER_COUNT / 39.37;
    //6.54 : 1
    inline constexpr double kDriveP = 15.0;
    inline constexpr double kDriveI = 0.01;
    inline constexpr double kDriveD = 0.1;
    inline constexpr double kDriveF = 0.2;

    inline constexpr double kAngleP = 0.30;
    inline constexpr double kAngleI = 0.002;
    inline constexpr double kAngleD = 0.0;
    inline constexpr double kAngleF = 0.0;

    inline constexpr int kCancoderCountsPerRotation = 4096;
    inline constexpr double kCancoderFeedbackCoefficient = 0.087890625;
}


namespace DrivetrainConstants 
{
    // should be -+, ++, --, +-
    inline constexpr frc::Translation2d frontLeftLocation {+0.4191_m, +0.4191_m}; 
    inline constexpr frc::Translation2d frontRightLocation {+0.4191_m, -0.4191_m};
    inline constexpr frc::Translation2d backLeftLocation {-0.4191_m, +0.4191_m};
    inline constexpr frc::Translation2d backRightLocation {-0.4191_m, -0.4191_m};

    inline constexpr units::meters_per_second_t MAX_SPEED = 4.084_mps;
    inline constexpr units::radians_per_second_t MAX_ANGULAR_SPEED{std::numbers::pi};

    inline constexpr pathplanner::HolonomicPathFollowerConfig holonomicConfig = pathplanner::HolonomicPathFollowerConfig (
        pathplanner::PIDConstants(SwerveModuleConstants::kDriveP, SwerveModuleConstants::kDriveI, SwerveModuleConstants::kDriveD), // Translation PID constants
        pathplanner::PIDConstants(SwerveModuleConstants::kAngleP, SwerveModuleConstants::kAngleI, SwerveModuleConstants::kAngleD), // Rotation PID constants
        MAX_SPEED, // Max module speed, in m/s
        0.351_m, // Drive base radius in meters. Distance from robot center to furthest module.
        pathplanner::ReplanningConfig() // Default path replanning config. See the API for the options here
    );

    inline constexpr double FL_DRIVE_ADJUSTMENT = 1.0;
    inline constexpr double FR_DRIVE_ADJUSTMENT = 1.0;
    inline constexpr double BL_DRIVE_ADJUSTMENT = 1.0;
    inline constexpr double BR_DRIVE_ADJUSTMENT = 1.0;

    inline constexpr double FL_OFFSET_DEGREES = 75.81;
    inline constexpr double FR_OFFSET_DEGREES = 146.30;
    inline constexpr double BL_OFFSET_DEGREES = -8.70;
    inline constexpr double BR_OFFSET_DEGREES = -90.60;

    inline constexpr double DRIVE_SLOW_ADJUSTMENT = 0.20;
}