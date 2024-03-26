#include "Controls.h"

Controls::Controls(Drivetrain *swerve)
{
    this->swerve = swerve;
}

void Controls::Periodic()
{   
    DriveControls();
}

void Controls::DriveControls()
{
    if (gamepad.GetXButton() == true)
        swerve->SetFieldRelative(true);
    if (gamepad.GetBButton() == true)
        swerve->SetFieldRelative(false);
    if (gamepad.GetYButton() == true)
    {
        swerve->ResetGyroAngle();
    }

    if (swerve->IsAlignmentOn())
        swerve->AlignSwerveDrive();
    else
    {
        swerve->DriveWithInput(gamepad.GetLeftY(), gamepad.GetLeftX(), gamepad.GetRightX(), gamepad.GetRightTriggerAxis() > 0.2);
    }
}