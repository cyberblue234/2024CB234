#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/PowerDistribution.h>
#include <frc/controller/PIDController.h>
#include "subsystems/Drivetrain.h"
#include "subsystems/Limelight.h"

extern frc::PowerDistribution pdp;
extern frc::XboxController gamePad;
extern frc::Joystick controls;

extern Drivetrain swerve;

extern Limelight limelight3;
extern Limelight limelight2;

extern frc::PIDController rotcontrol;
