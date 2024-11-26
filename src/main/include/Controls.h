#pragma once

#include <frc/XboxController.h>
#include "subsystems/Drivetrain.h"
#include "Constants.h"

#include <math.h>

class Controls
{
public:
    Controls(Drivetrain *);
    void Periodic(units::time::second_t period);
    void DriveControls(units::time::second_t period);

    frc::XboxController gamepad{0};

private:
    Drivetrain *swerve;
};