#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <numbers>

namespace RobotMap
{
    // TalonFX
    constexpr int FL_DRIVE_ADDRESS = 8;
    // TalonFX
    constexpr int FL_SWERVE_ADDRESS = 9;
    // CANCoder
    constexpr int FL_CANCODER_ADDRESS = 21;

    // TalonFX
    constexpr int FR_DRIVE_ADDRESS = 2;
    // TalonFX
    constexpr int FR_SWERVE_ADDRESS = 3;
    // CANCoder
    constexpr int FR_CANCODER_ADDRESS = 22;

    // TalonFX
    constexpr int BL_DRIVE_ADDRESS = 7;
    // TalonFX
    constexpr int BL_SWERVE_ADDRESS = 6;
    // CANCoder
    constexpr int BL_CANCODER_ADDRESS = 23;

    // TalonFX
    constexpr int BR_DRIVE_ADDRESS = 4;
    // TalonFX
    constexpr int BR_SWERVE_ADDRESS = 5;
    // CANCoder
    constexpr int BR_CANCODER_ADDRESS = 24;

    // Spark Flex
    constexpr int SHOOTER_MOTOR1_ADDRESS = 2;
    // Spakr Flex
    constexpr int SHOOTER_MOTOR2_ADDRESS = 3;
    // DIO
    constexpr int SHOOTER_ENCODER_ADDRESS = 0;

    // Spark Max
    constexpr int FEED_MOTOR_ADDRESS = 5;
    // DIO
    constexpr int TOP_FEED_SENSOR_ADDRESS = 1;
    // DIO
    constexpr int BOTTOM_FEED_SENSOR_ADDRESS = 2;

    // Spark Max
    constexpr int INTAKE_MOTOR_ADDRESS = 4;

    // TalonFX
    constexpr int ELEVATOR_MOTOR1_ADDRESS = 12;
    // TalonFX
    constexpr int ELEVATOR_MOTOR2_ADDRESS = 13;
    // DIO
    constexpr int ELEVATOR1_TOP_LIMIT_SWITCH = 3;
    // DIO
    constexpr int ELEVATOR1_BOTTOM_LIMIT_SWITCH = 4;
    // DIO
    constexpr int ELEVATOR2_TOP_LIMIT_SWITCH = 5;
    // DIO
    constexpr int ELEVATOR2_BOTTOM_LIMIT_SWITCH = 6;

}

namespace SwerveModuleConstants
{
    inline constexpr double ENCODER_INCHES_PER_COUNT = 0.00090689;

    inline constexpr double kDriveP = 0.5;                                                                                     
    inline constexpr double kDriveI = 0.0;
    inline constexpr double kDriveD = 0.1;
    inline constexpr double kDriveF = 0.2;

    inline constexpr double kAngleP = 15;
    inline constexpr double kAngleI = 0.0;
    inline constexpr double kAngleD = 0.5;
    inline constexpr double kAngleF = 0.0;

    // Wheel rotations to motor rotations
    inline constexpr double DRIVE_GEAR_RATIO = 8.75 / 1;
    // Circumference in meters
    inline constexpr double WHEEL_CIRCUMFERENCE = 0.32385;
}

namespace DrivetrainConstants
{
    inline constexpr frc::Translation2d frontLeftLocation{+0.2254_m, +0.2699_m};
    inline constexpr frc::Translation2d frontRightLocation{+0.2254_m, -0.2699_m};
    inline constexpr frc::Translation2d backLeftLocation{-0.3016_m, +0.2699_m};
    inline constexpr frc::Translation2d backRightLocation{-0.3016_m, -0.2699_m};

    inline constexpr units::meters_per_second_t MAX_SPEED = 4.084_mps;
    inline constexpr units::radians_per_second_t MAX_ANGULAR_SPEED{std::numbers::pi};

    inline constexpr pathplanner::HolonomicPathFollowerConfig holonomicConfig = pathplanner::HolonomicPathFollowerConfig(
        pathplanner::PIDConstants(SwerveModuleConstants::kDriveP, SwerveModuleConstants::kDriveI, SwerveModuleConstants::kDriveD), // Translation PID constants
        pathplanner::PIDConstants(SwerveModuleConstants::kAngleP, SwerveModuleConstants::kAngleI, SwerveModuleConstants::kAngleD), // Rotation PID constants
        MAX_SPEED,                                                                                                                 // Max module speed, in m/s
        0.4047_m,                                                                                                                   // Drive base radius in meters. Distance from robot center to furthest module.
        pathplanner::ReplanningConfig()                                                                                            // Default path replanning config. See the API for the options here
    );

    inline constexpr double FL_DRIVE_ADJUSTMENT = 1.0;
    inline constexpr double FR_DRIVE_ADJUSTMENT = 1.0;
    inline constexpr double BL_DRIVE_ADJUSTMENT = 1.0;
    inline constexpr double BR_DRIVE_ADJUSTMENT = 1.0;

    inline constexpr double FL_OFFSET_DEGREES = -0.666016;
    inline constexpr double FR_OFFSET_DEGREES = -0.78125;
    inline constexpr double BL_OFFSET_DEGREES = 0.8852546;
    inline constexpr double BR_OFFSET_DEGREES = -0.251953;

    inline constexpr double DRIVE_SLOW_ADJUSTMENT = 0.20;

    inline constexpr double kRotationP = 0.023;
    inline constexpr double kRotationI = 0.00;
    inline constexpr double kRotationD = 0.002;
}

namespace ShooterConstants
{
    inline constexpr double kShooterP = 0.000058;
    inline constexpr double kShooterI = 0.0;
    inline constexpr double kShooterD = 0.000022;
    inline constexpr double kShooterF = 0.000185;
}

namespace ElevatorConstants
{
    inline constexpr double kElevatorP = 0.15;
    inline constexpr double kElevatorI = 0.0;
    inline constexpr double kElevatorD = 0.0065;
    inline constexpr double kElevatorF = 0.0;

    inline constexpr double kForce = 41.5;
    inline constexpr double kKickup = 1.0;
    inline constexpr double kGravity = 9.8;

    inline constexpr double kSpeakerHeight = 1.35; // Speaker height to target in meters

    inline constexpr double SHOOTER_ANGLE_OFFSET = 15;
}

namespace ControlBoardConstants
{
    // Shoot button
    inline constexpr int SHOOT = 11;
    // Turn on shooter motors switch
    inline constexpr int SHOOTER_MOTORS = 10;
    // Intake from source switch
    inline constexpr int SOURCE_INTAKE = 9;
    // Intake from ground switch
    inline constexpr int GROUND_INTAKE = 12;
    // Move the elevator up switch
    inline constexpr int ELEVATOR_UP = 3;
    // Move the elevator down switch
    inline constexpr int ELEVATOR_DOWN = 4;
    // Toggle for the elevator to automatically move down
    inline constexpr int AUTO_ELEVATOR_DOWN = 6;
    // Purge any notes button
    inline constexpr int PURGE = 5;
    // Rotary switches: 0 - 9, easier to deal with than the raw analog inputs 
    // Scoring position at the subwoofer
    inline constexpr int POS_CLOSE = 0;
    // Scoring position directly back from the subwoofer
    inline constexpr int POS_MID = 1;
    // Scoring position with the bot touching the stage
    inline constexpr int POS_STAGE = 2;
    // Scoring position for the amp main
    inline constexpr int POS_AMP_MAIN = 3;
    // Scoring position for the amp speed 2
    inline constexpr int POS_AMP_2 = 7;
    // Scoring position for the amp speed 3
    inline constexpr int POS_AMP_3 = 8;
    // Auto alignment scoring position
    inline constexpr int AUTO_SCORE = 4;
    // Auto alignment with odometry scoring position
    inline constexpr int AUTO_ODOM_SCORE = 6;
    // Manual control for angle and elevator
    inline constexpr int MANUAL_SCORE = 5;
    // Manual control for amp scoring
    inline constexpr int MANUAL_AMP = 9;
}

namespace GameConstants
{
    inline constexpr frc::Pose2d blueSpeakerPose{0.50_m, 5.55_m, frc::Rotation2d(units::angle::degree_t(0.0))};
    inline constexpr frc::Pose2d redSpeakerPose{16.05_m, 5.55_m, frc::Rotation2d(units::angle::degree_t(180.0))};
}