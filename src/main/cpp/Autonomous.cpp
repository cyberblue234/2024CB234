#include "Autonomous.h"
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include "RobotExt.h"

using namespace pathplanner;

void Autonomous::ConfigureAuto() 
{
    onTheFly = frc2::cmd::RunOnce([this]() {
    frc::Pose2d currentPose = swerve.GetPose();

    // The rotation component in these poses represents the direction of travel
    frc::Pose2d startPos = frc::Pose2d(currentPose.Translation(), frc::Rotation2d());
    frc::Pose2d endPos = frc::Pose2d(currentPose.Translation() + frc::Translation2d(2.0_m, 0_m), frc::Rotation2d());

    std::vector<frc::Translation2d> bezierPoints = PathPlannerPath::bezierFromPoses({startPos, endPos});
    // Paths must be used as shared pointers
    auto path = std::make_shared<PathPlannerPath>(
      bezierPoints, 
      PathConstraints(4.0_mps, 4.0_mps_sq, 360_deg_per_s, 540_deg_per_s_sq),
      GoalEndState(0_mps, currentPose.Rotation())
    );

    // Prevent this path from being flipped on the red alliance, since the given positions are already correct
    path->preventFlipping = true;

    this->followOnTheFly = AutoBuilder::followPath(path).Unwrap();
    this->followOnTheFly->Schedule();
  }).Unwrap();
  onTheFly->Schedule();
}