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

        void LEDControls(ControlMethods);
        void SetLEDs(LEDConstants::SetLEDs set) { candle.SetLEDs(set.r, set.g, set.b, set.w, set.startIndex, set.count); };
    private:
        ctre::phoenix::led::CANdle candle{RobotMap::CANDLE_ADDRESS};

        void LEDsOff() { SetLEDs(LEDConstants::kOff); };
        void Intaking() { SetLEDs(LEDConstants::kIntaking); };
        void NoteSecured() { SetLEDs(LEDConstants::kNoteSecured); };
        void ElevatorDown() { SetLEDs(LEDConstants::kElevatorDown); };
        void ElevatorUp() { SetLEDs(LEDConstants::kElevatorUp); };
};