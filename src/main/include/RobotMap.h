#ifndef _ROBOTMAP_H
#define _ROBOTMAP_H

// Class             MyClass
// Object            myClassObject
// Local variable    local_variable
// Constant/Define   DRIVE_SPEED
// Enumeration       {kOpen, kClose}

constexpr double FEET_PER_METER = 3.28048;

// CAN Addresses and Constants

constexpr int FL_DRIVE_ADDRESS = 8;
constexpr int FL_SWERVE_ADDRESS = 9;
constexpr int FL_CANCODER_ADDRESS = 21;
constexpr double FL_OFFSET_DEGREES = 75.81; //-105.81;

constexpr int FR_DRIVE_ADDRESS = 2;
constexpr int FR_SWERVE_ADDRESS = 3;
constexpr int FR_CANCODER_ADDRESS = 22;
constexpr double FR_OFFSET_DEGREES = 146.30; //-34.30;

constexpr int BL_DRIVE_ADDRESS = 7;
constexpr int BL_SWERVE_ADDRESS = 6;
constexpr int BL_CANCODER_ADDRESS = 23;
constexpr double BL_OFFSET_DEGREES = -8.70;

constexpr int BR_DRIVE_ADDRESS = 4;
constexpr int BR_SWERVE_ADDRESS = 5;
constexpr int BR_CANCODER_ADDRESS = 24;
constexpr double BR_OFFSET_DEGREES = -90.60;

constexpr int ARM_MOTOR_ADDRESS = 99;
constexpr int ROTATOR_MOTOR_ADDRESS = 12;
constexpr int GRIPPER_MOTOR_ADDRESS = 13;

constexpr int ARM_PCM_ADDRESS = 0;
constexpr int LED_PCM_ADDRESS = 1;

// roboRIO Input Channels

constexpr int ARM_ENCODER_CHANNEL = 0;     // Analog input
constexpr int ROTATOR_ENCODER_CHANNEL = 1; // Analog input
constexpr int GRIPPER_ENCODER_CHANNEL = 2; // Analog input
constexpr int PRESSURE_SENSOR_CHANNEL = 3;
constexpr int ROTATOR_LIMIT_SWITCH_CHANNEL = 9; // DIgital input

// Operating board switch codes

constexpr int DRIVE_ANCHOR_SWITCH = 6;

constexpr int TOWER_HOME_SWITCH = 4;
constexpr int TOWER_FLOOR_SWITCH = 11;
constexpr int TOWER_SHELF_SWITCH = 9;
constexpr int TOWER_FLIP_SWITCH = 3;
constexpr int TOWER_SCORE_HIGH_SWITCH = 8;
constexpr int TOWER_SCORE_MID_SWITCH = 12;
constexpr int TOWER_GRIPPER_SWITCH_OPEN = 7;
constexpr int TOWER_GRIPPER_SWITCH_CLOSE = 10;
constexpr int TOWER_LIMITS_OFF_SWITCH = 5;
constexpr int TOWER_CUBE_SELECT_SWITCH = 2;
constexpr int TOWER_CONE_SELECT_SWITCH = 1;

constexpr double DRIVE_SLOW_ADJUSTMENT = 0.20;

#endif
