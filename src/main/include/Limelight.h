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
    double GetTotalLatency() { return pipeline_latency + capture_latency;}
    void SetLEDMode(LEDMode);
    void SetCamMode(CamMode);
    void SetPipelineID(PipelineID);
    auto getDistanceFromTarget();
    frc::Pose2d getRobotPose();

private:
    double target_x;
    double target_y;
    double target_area;
    double target_valid;
    double pipeline_id;
    double april_tag_id;
    double pipeline_latency;
    double capture_latency;

    std::vector<double> botpose;
    std::vector<double> botpose_blue;
    std::vector<double> botpose_red;
    
    std::vector<double> targetpose_robotspace;
};

#endif