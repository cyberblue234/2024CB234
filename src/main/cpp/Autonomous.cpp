#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include "Autonomous.h"
#include "RobotExt.h"

Autonomous::Autonomous()
{
    autoTimer.Start();
    autoLogTimer.Start();
}

void Autonomous::AutoInit()
{
    swerve.ResetGyroForAuto();
    swerve.ResetGyroPitch();
    swerve.ResetGyroRoll();
    swerve.ResetDriveEncoders();
    swerve.SetDriveOpenLoopRamp(2.0);
}

frc2::CommandPtr Autonomous::GetAutonomousCommand()
{
    return pathplanner::PathPlannerAuto("Test Auto").ToPtr();
}

void Autonomous::LogAutoData()
{
#define AUTO_COUNT 750

    static long count = 0;

    double time = (double)autoLogTimer.Get();

    if (count == 0)
    {
        a_output = fopen("/cb234/autodata.csv", "w");
        autoLogTimer.Reset();
    }

    if (count < AUTO_COUNT)
    {
        if (a_output != NULL)
        {
            if (count == 0)
            {
                fprintf(a_output, "MSEC \r\n");
            }
            fprintf(a_output, "%10.5f\r\n", time);
        }
    }
    if (a_output != NULL && count == AUTO_COUNT)
    {
        fflush(a_output);
    }
    count++;
}