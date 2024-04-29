#pragma once

#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include "Constants.h"

class Controls
{
public:
    Controls()
    {
        gamepad = frc::XboxController(0);
        controlBoard = frc::Joystick(1);
    }

    static int GetSelectedRotaryIndex() { return AnalogToRotaryIndex(controlBoard.GetX()); }

    static bool Shoot() { return controlBoard.GetRawButton(ControlBoardConstants::SHOOT); };
    static bool ShooterMotors() { return controlBoard.GetRawButton(ControlBoardConstants::SHOOTER_MOTORS); };
    static bool SourceIntake() { return controlBoard.GetRawButton(ControlBoardConstants::SOURCE_INTAKE); };
    static bool GroundIntake() { return controlBoard.GetRawButton(ControlBoardConstants::GROUND_INTAKE); };
    static bool ElevatorUp() { return controlBoard.GetRawButton(ControlBoardConstants::ELEVATOR_UP); };
    static bool ElevatorDown() { return controlBoard.GetRawButton(ControlBoardConstants::ELEVATOR_DOWN); };
    static bool AutoElevatorDown() { return controlBoard.GetRawButton(ControlBoardConstants::AUTO_ELEVATOR_DOWN); };
    static bool Purge() { return controlBoard.GetRawButton(ControlBoardConstants::PURGE); };

    static bool Trap() { return GetSelectedRotaryIndex() == ControlBoardConstants::TRAP; };
    static bool Mid() { return GetSelectedRotaryIndex() == ControlBoardConstants::MID; };
    static bool Stage() { return GetSelectedRotaryIndex() == ControlBoardConstants::STAGE; };
    static bool AmpMain() { return GetSelectedRotaryIndex() == ControlBoardConstants::AMP_MAIN; };
    static bool Amp2() { return GetSelectedRotaryIndex() == ControlBoardConstants::AMP_2; };
    static bool Amp3() { return GetSelectedRotaryIndex() == ControlBoardConstants::AMP_3; };
    static bool Amp4() { return GetSelectedRotaryIndex() == ControlBoardConstants::AMP_4; };
    static bool AutoScore() { return GetSelectedRotaryIndex() == ControlBoardConstants::AUTO_SCORE; };
    static bool ManualScore() { return GetSelectedRotaryIndex() == ControlBoardConstants::MANUAL_SCORE; };
    static bool ManualAmp() { return GetSelectedRotaryIndex() == ControlBoardConstants::MANUAL_AMP; };

    static bool AmpShot() { return AmpMain() || Amp2() || Amp3() || Amp4() || ManualAmp(); };
    static bool TrapShot() { return Trap(); }
    static bool SpeakerShot() { return Mid() || Stage() || AutoScore() || ManualScore(); };

    static void RumbleGamepad() { gamepad.SetRumble(gamepad.kBothRumble, 1.0); };
    static void StopRumble() { gamepad.SetRumble(gamepad.kBothRumble, 0.0); };

    static frc::XboxController gamepad;
    static frc::Joystick controlBoard;

private:
    static int AnalogToRotaryIndex(double analogInput) 
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
};