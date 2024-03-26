#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>

#include <wpinet/PortForwarder.h>

#include "RobotContainer.h"
#include "Constants.h"

class Robot : public frc::TimedRobot
{

public:
    void RobotInit() override;
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestPeriodic() override;
    void SimulationInit() override;
    void SimulationPeriodic() override;

private:
    std::optional<frc2::CommandPtr> autonomousCommand;
    RobotContainer container;
};