#pragma once

#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc2/command/SubsystemBase.h>
#include "subsystems/Drivetrain.h"

class Controls : frc2::SubsystemBase
{
public:
    Controls(Drivetrain *);
    void Periodic() override;
    void DriveControls();

    frc::XboxController gamepad{0};

private:
    Drivetrain *swerve;
};