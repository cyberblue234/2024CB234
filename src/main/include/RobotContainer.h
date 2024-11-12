#pragma once

#include <string>
#include <optional>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PowerDistribution.h>

#include "subsystems/Drivetrain.h"
#include "Controls.h"

class RobotContainer
{
public:
	RobotContainer();

	void Periodic(units::time::second_t period);

	Drivetrain *GetSwerve() { return &swerve; };


private:
	Drivetrain swerve;

	frc::PowerDistribution pdp;

	Controls controls;
};
