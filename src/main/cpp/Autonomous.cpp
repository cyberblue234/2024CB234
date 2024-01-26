#include <pathplanner/lib/commands/PathPlannerAuto.h>

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
    double volts = pdp.GetVoltage();

    double pitch = swerve.GetGyroPitch();
    double roll = swerve.GetGyroRoll();
    double angle = swerve.GetGyroAngle();

    double speed = swerve.GetDriveRPM() * 0.002234;
    double fl_rpm = swerve.GetLeftDriveRPM();
    double fr_rpm = swerve.GetRightDriveRPM();
    double bl_rpm = swerve.GetBackLeftDriveRPM();
    double br_rpm = swerve.GetBackRightDriveRPM();

    double fl_angle = swerve.GetFrontLeftAngle();
    double fr_angle = swerve.GetFrontRightAngle();
    double bl_angle = swerve.GetBackLeftAngle();
    double br_angle = swerve.GetBackRightAngle();

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
                fprintf(a_output, "MSEC,VOLTS,STATE,PITCH,ROLL,ANGLE,DIST,SPEED,FL_RPM,FR_RPM,BL_RPM,BR_RPM,FL_ANGLE,FR_ANGLE,BL_ANGLE,BR_ANGLE \r\n");
            }
            fprintf(a_output, "%10.5f,%7.3f,%5ld,%6.1f,%6.1f,%6.1f,%6.1f,%6.1f,%6.1f,%6.1f,%6.1f,%6.1f,%6.1f,%6.1f,%6.1f,%6.1f\r\n", time, volts, state, pitch, roll, angle, distance, speed, fl_rpm, fr_rpm, bl_rpm, br_rpm, fl_angle, fr_angle, bl_angle, br_angle);
        }
    }
    if (a_output != NULL && count == AUTO_COUNT)
    {
        fflush(a_output);
    }
    count++;
}