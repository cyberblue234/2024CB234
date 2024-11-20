#include "Controls.h"

Controls::Controls(Drivetrain *swerve)
{
    this->swerve = swerve;

}

void Controls::Periodic(units::time::second_t period)
{
    DriveControls(period);
}

void Controls::DriveControls(units::time::second_t period)
{
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -xSpeedLimiter.Calculate(
                            frc::ApplyDeadband(gamepad.GetLeftY(), 0.08)) *
                        DrivetrainConstants::kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed = -ySpeedLimiter.Calculate(
                            frc::ApplyDeadband(gamepad.GetLeftX(), 0.08)) *
                        DrivetrainConstants::kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const auto rot = -rotLimiter.Calculate(
                         frc::ApplyDeadband(gamepad.GetRightX(), 0.08)) *
                     DrivetrainConstants::kMaxAngularSpeed;

    swerve->Drive(xSpeed, ySpeed, rot, true, period);
}