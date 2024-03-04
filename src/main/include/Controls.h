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

    void SetSelectedRotaryIndex(int newIndex) { selectedRotaryIndex = newIndex; };
    int GetSelectedRotaryIndex() { return selectedRotaryIndex; };
    int AnalogToRotaryIndex(double analogInput) 
    { 
        if (analogInput < -0.85) return 0;
        if (analogInput < -0.65) return 1;
        if (analogInput < -0.30) return 2;
        if (analogInput < -0.10) return 3;
        if (analogInput <  0.10) return 4;
        if (analogInput <  0.30) return 5;
        if (analogInput <  0.50) return 6;
        if (analogInput <  0.70) return 7;
        if (analogInput <  0.90) return 8;
        if (analogInput <  1.10) return 9;
        return -1;
    }; 

    void RumbleGamepad() { gamepad.SetRumble(gamepad.kBothRumble, 1.0); };

    frc::XboxController gamepad{0};
    frc::Joystick controlBoard{1};

private:
    Drivetrain *swerve;
    Shooter *shooter;
    Intake *intake;
    Feeder *feeder;
    Elevator *elevator;
    Limelight *limelight3;

    int selectedRotaryIndex = 0;
};