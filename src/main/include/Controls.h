#pragma once

#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc2/command/SubsystemBase.h>
#include "subsystems/Drivetrain.h"
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"
#include "subsystems/Feeder.h"
#include "subsystems/Elevator.h"
#include "subsystems/Limelight.h"

class Controls : frc2::SubsystemBase
{
public:
    Controls(Drivetrain *, Shooter *, Intake *, Elevator *, Feeder *, Limelight *);
    void Periodic() override;
    void DriveControls();
    void ShooterControls();
    void IntakeControls();
    void ElevatorControls();
    void FeederControls();

    frc::XboxController gamepad{0};
    frc::Joystick controlsBoard{1};

private:
    Drivetrain *swerve;
    Shooter *shooter;
    Intake *intake;
    Feeder *feeder;
    Elevator *elevator;
    Limelight *limelight3;
};