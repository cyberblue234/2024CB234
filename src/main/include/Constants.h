#pragma once

#include <frc/geometry/Translation2d.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/voltage.h>
#include <numbers>
#include <string>
#include <array>

namespace RobotMap
{
    // TalonFX
    constexpr int kFrontLeftDriveID = 8;
    // TalonFX
    constexpr int kFrontLeftTurnID = 9;
    // CANCoder
    constexpr int kFrontLeftCanCoderID = 21;

    // TalonFX
    constexpr int kFrontRightDriveID = 2;
    // TalonFX
    constexpr int kFrontRightTurnID = 3;
    // CANCoder
    constexpr int kFrontRightCanCoderID = 22;

    // TalonFX
    constexpr int kBackLeftDriveID = 7;
    // TalonFX
    constexpr int kBackLeftTurnID = 6;
    // CANCoder
    constexpr int kBackLeftCanCoderID = 23;

    // TalonFX
    constexpr int kBackRightDriveID = 4;
    // TalonFX
    constexpr int kBackRightTurnID = 5;
    // CANCoder
    constexpr int kBackRightCanCoderID = 24;
}

namespace SwerveModuleConstants
{
    inline constexpr double kDriveP = 0.50;
    inline constexpr double kDriveI = 0.0;
    inline constexpr double kDriveD = 0.1;
    inline constexpr auto kDrive_kS = 1_V;
    inline constexpr auto kDrive_kV = 0.5_V / 1_tps;

    inline constexpr double kTurnP = 0.1;
    inline constexpr double kTurnI = 0.0;
    inline constexpr double kTurnD = 0.5;

    inline constexpr double kDriveGearRatio = 6.54;
    inline constexpr double kTurnGearRatio = 11.31;
    inline constexpr double kWheelRadius = 0.0491;
    inline constexpr double kDriveDistanceRatio = kWheelRadius * 2 * std::numbers::pi / kDriveGearRatio;
    inline constexpr double kTurnDistanceRatio = 2 * std::numbers::pi;

    inline constexpr units::radians_per_second_t kModuleMaxAngularVelocity = std::numbers::pi * 4_rad_per_s;
    inline constexpr units::radians_per_second_squared_t kModuleMaxAngularAcceleration = std::numbers::pi * 2_rad_per_s / 1_s;

    inline constexpr units::volt_t kVoltageComp = 11.5_V;
}

namespace DrivetrainConstants
{
    inline constexpr frc::Translation2d kFrontLeftLocation{+0.2254_m, +0.2699_m};
    inline constexpr frc::Translation2d kFrontRightLocation{+0.2254_m, -0.2699_m};
    inline constexpr frc::Translation2d kBackLeftLocation{-0.3016_m, +0.2699_m};
    inline constexpr frc::Translation2d kBackRightLocation{-0.3016_m, -0.2699_m};

    inline constexpr units::meters_per_second_t kMaxSpeed = 6.0_mps;
    inline constexpr units::radians_per_second_t kMaxAngularSpeed = std::numbers::pi * 4_rad_per_s;

    inline constexpr double kFrontLeftMagnetOffset  = 0.0; //-0.666016;
    inline constexpr double kFrontRightMagnetOffset = 0.0; //-0.78125;
    inline constexpr double kBackLeftMagnetOffset   = 0.0; //0.8852546;
    inline constexpr double kBackRightMagnetOffset  = 0.0; //-0.251953;

    inline constexpr double kDriveSlowAdjustment = 0.20;
}

template <typename T>
constexpr T clamp(T val, T low, T high) 
{
    return val > low && val < high ? val : val <= low ? low : high; 
};