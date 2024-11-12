#include "RobotContainer.h"

RobotContainer::RobotContainer() : swerve(),
								   pdp(1, frc::PowerDistribution::ModuleType::kRev),
								   controls(GetSwerve()) {}

void RobotContainer::Periodic(units::time::second_t period)
{
	controls.Periodic(period);
}