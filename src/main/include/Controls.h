#pragma once

#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include "subsystems/Drivetrain.h"
#include "subsystems/simulation/SimulatedDrivetrain.h"
#include <frc/filter/SlewRateLimiter.h>
#include "Constants.h"

#include <math.h>

class Controls
{
public:
    Controls(Drivetrain *);
    Controls(SimulatedDrivetrain *);
    void Periodic(units::time::second_t period);
    void DriveControls(units::time::second_t period);

    frc::XboxController gamepad{0};

private:
    Drivetrain *swerve;
    SimulatedDrivetrain *swerveSim;
};