#pragma once

#include <frc/geometry/Translation2d.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <numbers>

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

    constexpr int SHOOTER_MOTOR1_ADDRESS = 2;
    constexpr int SHOOTER_MOTOR2_ADDRESS = 3;
    constexpr int SHOOTER_ENCODER_ADDRESS = 1;

    constexpr int FEED_MOTOR_ADDRESS = 5;
    constexpr int FEED_SENSOR_ADDRESS = 2;

    constexpr int INTAKE_MOTOR_ADDRESS = 4;

    constexpr int ELEVATOR_MOTOR1_ADDRESS = 6;
    constexpr int ELEVATOR_MOTOR2_ADDRESS = 7;
}

namespace SwerveModuleConstants
{
    inline constexpr double ENCODER_INCHES_PER_COUNT = 0.00090689;
    inline constexpr double ENCODER_METERS_PER_COUNT = ENCODER_INCHES_PER_COUNT / 39.37;
    // 6.54 : 1
    inline constexpr double kDriveP = 3.0;
    inline constexpr double kDriveI = 0.0;
    inline constexpr double kDriveD = 0.5250;
    inline constexpr double kDriveF = 0.2;

    inline constexpr double kAngleP = 7.5; // 030;
    inline constexpr double kAngleI = 0.0;
    inline constexpr double kAngleD = 0.5; // 0.001;
    inline constexpr double kAngleF = 0.0;

    inline constexpr int kCancoderCountsPerRotation = 16384;
    inline constexpr double kCancoderFeedbackCoefficient = 0.087890625;
    inline constexpr double kSwerveModuleGearRatio = 15.4;
}

namespace DrivetrainConstants
{
    // should be -+, ++, --, +-
    inline constexpr frc::Translation2d frontLeftLocation{+0.4191_m, +0.4191_m};
    inline constexpr frc::Translation2d frontRightLocation{+0.4191_m, -0.4191_m};
    inline constexpr frc::Translation2d backLeftLocation{-0.4191_m, +0.4191_m};
    inline constexpr frc::Translation2d backRightLocation{-0.4191_m, -0.4191_m};

    inline constexpr units::meters_per_second_t MAX_SPEED = 4.084_mps;
    inline constexpr units::radians_per_second_t MAX_ANGULAR_SPEED{std::numbers::pi};

    inline constexpr pathplanner::HolonomicPathFollowerConfig holonomicConfig = pathplanner::HolonomicPathFollowerConfig(
        pathplanner::PIDConstants(SwerveModuleConstants::kDriveP, SwerveModuleConstants::kDriveI, SwerveModuleConstants::kDriveD), // Translation PID constants
        pathplanner::PIDConstants(SwerveModuleConstants::kAngleP, SwerveModuleConstants::kAngleI, SwerveModuleConstants::kAngleD), // Rotation PID constants
        MAX_SPEED,                                                                                                                 // Max module speed, in m/s
        0.351_m,                                                                                                                   // Drive base radius in meters. Distance from robot center to furthest module.
        pathplanner::ReplanningConfig()                                                                                            // Default path replanning config. See the API for the options here
    );

    inline constexpr double FL_DRIVE_ADJUSTMENT = 1.0;
    inline constexpr double FR_DRIVE_ADJUSTMENT = 1.0;
    inline constexpr double BL_DRIVE_ADJUSTMENT = 1.0;
    inline constexpr double BR_DRIVE_ADJUSTMENT = 1.0;

    inline constexpr double FL_OFFSET_DEGREES = 0.445;  //-0.2739; //-0.2109;  //0.664;  //75.81;
    inline constexpr double FR_OFFSET_DEGREES = 0.149;  //-0.0979; //-0.0962;  //0.031;  //146.30;
    inline constexpr double BL_OFFSET_DEGREES = -0.041; //-0.5425; //0.0222;  //0.601;   //-8.70;
    inline constexpr double BR_OFFSET_DEGREES = -0.256; //-0.7263; //-0.2495;  //-0.171;   //-90.60;

    inline constexpr double DRIVE_SLOW_ADJUSTMENT = 0.20;

    inline constexpr double kRotationP = 0.025;
    inline constexpr double kRotationI = 0.00;
    inline constexpr double kRotationD = 0.002;
}

namespace ShooterConstants
{
    inline constexpr double kShooterP = 0.000058;
    inline constexpr double kShooterI = 0.0;
    inline constexpr double kShooterD = 0.000022;
    inline constexpr double kShooterF = 0.000158;

    inline constexpr double SHOOTER_ANGLE_OFFSET = 0.0;
}

namespace ElevatorConstants
{
    inline constexpr double kElevatorP = 0.0;
    inline constexpr double kElevatorI = 0.0;
    inline constexpr double kElevatorD = 0.0;
    inline constexpr double kElevatorF = 0.0;

    inline constexpr double kForce = 45.0;
    inline constexpr double kKickup = 1.0;
    inline constexpr double kGravity = 9.8;

    inline constexpr double kSpeakerHeight = 1.0914; // Speaker height to target in meters
}

namespace ControlBoardConstants
{
    // Shoot button
    inline constexpr int SHOOT = 10;
    // Turn on shooter motors switch
    inline constexpr int SHOOTER_MOTORS = 9;
    // Intake from source switch
    inline constexpr int SOURCE_INTAKE = 8;
    // Intake from ground switch
    inline constexpr int GROUND_INTAKE = 11;
    // Move the elevator up switch
    inline constexpr int ELEVATOR_UP = 2;
    // Move the elevator down switch
    inline constexpr int ELEVATOR_DOWN = 3;
    // Turn the anchor on switch
    inline constexpr int ANCHOR = 5;
    // Purge any notes button
    inline constexpr int PURGE = 4;
    // Rotary switches: 0 - 9, easier to deal with than the raw analog inputs 
    // Scoring position at the subwoofer
    inline constexpr int POS_CLOSE = 0;
    // Scoring position directly back from the subwoofer
    inline constexpr int POS_MID = 1;
    // Scoring position with the bot touching the stage
    inline constexpr int POS_STAGE = 2;
    // Scoring position for the amp
    inline constexpr int POS_AMP = 3;
    // Auto alignment scoring position
    inline constexpr int AUTO_SCORE = 4;
    // Scoring position for the trap
    inline constexpr int POS_TRAP = 7;
    // Manual control for angle and elevator
    inline constexpr int MANUAL_SCORE = 9;

    inline constexpr int AnalogToRotaryIndex(double analogInput)
    {
        return (int) (analogInput + 0.05) * 4.5 + 4.5;
    }
}