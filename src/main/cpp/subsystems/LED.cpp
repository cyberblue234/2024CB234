#include "subsystems/LED.h"
#include <frc/smartdashboard/SmartDashboard.h>

void LED::LEDControls(LED::ControlMethods method)
{
    switch (method)
    {
        case LED::ControlMethods::kOff:
            LEDsOff();
            break;
        case LED::ControlMethods::kIntaking:
            Intaking();
            break;
        case LED::ControlMethods::kNoteSecured:
            NoteSecured();
            break;
        case LED::ControlMethods::kElevatorDown:
            ElevatorDown();
            break;
        case LED::ControlMethods::kElevatorUp:
            ElevatorUp();
            break;
        default:
            LEDsOff();
            break;
    }
}