#ifndef _ROBOTEXT_H
#define _ROBOTEXT_H

#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/PowerDistribution.h>
#include <frc/DriverStation.h>
#include "Drivetrain.h"

extern frc::PowerDistribution pdp;
extern frc::XboxController gamePad;
extern frc::Joystick controls;

extern Drivetrain swerve;

#endif