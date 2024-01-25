// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Translation2d.h>
#include <units/velocity.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>

namespace DrivetrainConstants 
{

inline constexpr frc::Translation2d frontLeftLocation{+0.4191_m, +0.4191_m}; 
inline constexpr frc::Translation2d frontRightLocation{+0.4191_m, -0.4191_m};
inline constexpr frc::Translation2d backLeftLocation{-0.4191_m, +0.4191_m};
inline constexpr frc::Translation2d backRightLocation{-0.4191_m, -0.4191_m};

inline constexpr units::meters_per_second_t MAX_SPEED = 4.084_mps;
inline constexpr units::radians_per_second_t MAX_ANGULAR_SPEED{std::numbers::pi};

inline constexpr pathplanner::HolonomicPathFollowerConfig holonomicConfig  = pathplanner::HolonomicPathFollowerConfig(
    pathplanner::PIDConstants(SwerveModule::kDriveP, SwerveModule::kDriveI, SwerveModule::kDriveD), // Translation PID constants
    pathplanner::PIDConstants(SwerveModule::kAngleP, SwerveModule::kAngleI, SwerveModule::kAngleD), // Rotation PID constants
    MAX_SPEED, // Max module speed, in m/s
    frontLeftLocation.Norm(), // Drive base radius in meters. Distance from robot center to furthest module.
    pathplanner::ReplanningConfig() // Default path replanning config. See the API for the options here
);

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

inline constexpr kCancoderCountsPerRotation = 4096;
inline constexpr kCancoderFeedbackCoefficient = 0.087890625;

}
