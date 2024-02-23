#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include "subsystems/Drivetrain.h"
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"
#include "subsystems/Feeder.h"
#include "subsystems/Elevator.h"

class Controls
{
public:
    void ControlsPeriodic(const Drivetrain &swerve, const Shooter &shooter, const Intake &intake, const Feeder &feeder, const Elevator &elevator);
    void DriveControls(const Drivetrain &swerve);
    void ShooterControls(const Shooter &shooter);
    void IntakeControls(const Intake &intake);
    void ElevatorControls(const Elevator &elevator);
    void FeederControls(const Shooter &shooter, const Elevator &elevator, const Feeder &feeder);

    frc::XboxController gamepad{0};
    frc::Joystick controlsBoard{1};
private:
}