#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PowerDistribution.h>
#include <frc/DriverStation.h>

#include "ctre/Phoenix.h"

#include "Drivetrain.h"
#include "Teleop.h"
#include "Autonomous.h"
#include "Limelight.h"

frc::PowerDistribution pdp{1, frc::PowerDistribution::ModuleType::kRev};
frc::XboxController gamePad{0};
frc::Joystick controls(1);

Teleop teleop;
Autonomous autonomous;
Drivetrain swerve;
Limelight limelight3("limelight");

class Robot : public frc::TimedRobot
{
public:
    void RobotInit() override
    {
        swerve.ResetGyroPitch();
        swerve.ResetGyroRoll();
        swerve.ResetGyroAngle();
    }

    void RobotPeriodic() override
    {
        frc2::CommandScheduler::GetInstance().Run();
    }

    void AutonomousInit() override
    {
        
    }

    void AutonomousPeriodic() override
    {
        
    }

    void TeleopInit() override
    {
        teleop.TeleopInit();
    }

    void TeleopPeriodic() override
    {
        teleop.OperatorControls();
    }

    void DisabledInit() override {}

    void DisabledPeriodic() override {}

private:
};

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
