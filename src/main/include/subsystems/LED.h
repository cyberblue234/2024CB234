#pragma once

#include <ctre/phoenix/led/CANdle.h>
#include "Constants.h"

class LED
{
    public:
        enum ControlMethods
        {
            kOff,
            kIntaking,
            kNoteSecured,
            kElevatorDown,
            kElevatorUp
        };

        static void LEDControls(ControlMethods);
        static void SetLEDs(LEDConstants::SetLEDs set) { candle.SetLEDs(set.r, set.g, set.b, set.w, set.startIndex, set.count); };
    private:
        static ctre::phoenix::led::CANdle candle{RobotMap::CANDLE_ADDRESS};

        static void LEDsOff() { SetLEDs(LEDConstants::kOff); };
        static void Intaking() { SetLEDs(LEDConstants::kIntaking); };
        static void NoteSecured() { SetLEDs(LEDConstants::kNoteSecured); };
        static void ElevatorDown() { SetLEDs(LEDConstants::kElevatorDown); };
        static void ElevatorUp() { SetLEDs(LEDConstants::kElevatorUp); };
};