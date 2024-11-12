#pragma once

#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include "subsystems/Drivetrain.h"
#include <frc/filter/SlewRateLimiter.h>
#include "Constants.h"

class Controls
{
public:
    Controls(Drivetrain *);
    void Periodic(units::time::second_t period);
    void DriveControls(units::time::second_t period);

    frc::XboxController gamepad{0};

private:
    Drivetrain *swerve;

    frc::SlewRateLimiter<units::scalar> xSpeedLimiter{3 / 1_s};
    frc::SlewRateLimiter<units::scalar> ySpeedLimiter{3 / 1_s};
    frc::SlewRateLimiter<units::scalar> rotLimiter{3 / 1_s};
};