
#include "RobotExt.h"
#include "Limelight.h"


void Limelight::UpdateLimelightTracking()
{
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    target_x = table->GetNumber("tx", 0.0);
    target_y = table->GetNumber("ty", 0.0);
    target_area = table->GetNumber("ta", 0.0);
    target_skew = table->GetNumber("ts", 0.0);
    april_tag_id = table->GetNumber("tid", 0);

    bot_pose = table->GetNumberArray("botpose", std::vector<double>(6));
    targetpose_robotspace = table->GetNumberArray("targetpose_robotspace", std::vector<double>(6));

    frc::SmartDashboard::PutNumber("TX", target_x);
    frc::SmartDashboard::PutNumber("TY", target_y);
    frc::SmartDashboard::PutNumber("TA", target_area);
    frc::SmartDashboard::PutNumber("APRIL_TAG_ID", april_tag_id);
}

void Limelight::SetPipelineID(PipelineID pid)
{
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline", int(pid));
}

void Limelight::SetLEDMode(LEDMode m)
{
    int led_mode = m;
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", led_mode);
}

void Limelight::SetCamMode(CamMode m)
{
    int cam_mode = m;
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", cam_mode);
}

auto Limelight::getDistanceFromTarget()
{
    double yDistance = targetpose_robotspace.at(0);
    double zDistance = targetpose_robotspace.at(2);

    auto distance = units::meter_t(sqrt((yDistance*yDistance)+(zDistance*zDistance)));

    return distance;
}

// frc::Pose2d Limelight::getRobotPose()
// {

//     return
// }