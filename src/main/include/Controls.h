#pragma once

#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/Drivetrain.h"
#include "Constants.h"

#include <math.h>

class Controls
{
public:
    /// @brief Constructs the Controls object to control the provided subsystems 
    /// @param drivetrain pointer to the drivetrain object
    Controls(Drivetrain *);
    /// @brief Runs all of the subsystems controls every cycle
    /// @param period
    void Periodic(units::time::second_t period);
    /// @brief Drivetrain controls
    /// @param period
    void DriveControls(units::time::second_t period);

    /// @brief Applies a deadband around zero. Zone depends on deadband value. 
    /// @param value Value to apply the deadband to
    /// @param deadband Value around zero
    /// @return value or 0
    double ApplyDeadband(double value, double deadband)
    {
        if (std::abs(value) < deadband) return 0;
        return value;
    }

    frc::XboxController gamepad{0};

private:
    Drivetrain *swerve;
};