#ifndef _LIMELIGHT_H
#define _LIMELIGHT_H

#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "RobotMap.h"


class Limelight
{
public:
    enum LEDMode
    {
        kLEDPipline,
        kLEDOff,
        kLEDBlink,
        kLEDOn
    };
    enum CamMode
    {
        kVisonProcessor,
        kDriverCamera
    };
    enum PipelineID
    {
        kAprilTag,
        kReflectiveTape
    };


    void UpdateLimelightTracking();
    double GetTargetX() { return target_x; };
    double GetTargetY() { return target_y; };
    double GetTargetArea() { return target_area; };
    double GetAprilTagID() { return april_tag_id; };
    void SetLEDMode(LEDMode);
    void SetCamMode(CamMode);
    void SetPipelineID(PipelineID);
    auto getDistanceFromTarget();
    // frc::Pose2d Limelight::getRobotPose();

private:
    double target_x;
    double target_y;
    double target_area;
    double target_skew;
    double pipeline_id;
    double april_tag_id;

    std::vector<double> bot_pose;
    std::vector<double> targetpose_robotspace;
};

#endif