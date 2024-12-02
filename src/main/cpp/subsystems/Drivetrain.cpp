#include "subsystems/Drivetrain.h"

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative,
                       units::second_t period)
{
    frc::SmartDashboard::PutNumber("xSpeed", xSpeed.value());
    frc::SmartDashboard::PutNumber("ySpeed", ySpeed.value());
    frc::SmartDashboard::PutNumber("rot", rot.value());
    
    auto states =
        kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
            fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                                xSpeed, ySpeed, rot, gyro.GetRotation2d())
                          : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
            period));

    kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

    auto [fl, fr, bl, br] = states;

    frontLeft.SetDesiredState(fl);
    frontRight.SetDesiredState(fr);
    backLeft.SetDesiredState(bl);
    backRight.SetDesiredState(br);

    // frontLeft.SetDesiredState( {4_mps, frc::Rotation2d(units::angle::degree_t(45))});
    // frontRight.SetDesiredState({4_mps, frc::Rotation2d(units::angle::degree_t(45))});
    // backLeft.SetDesiredState(  {4_mps, frc::Rotation2d(units::angle::degree_t(45))});
    // backRight.SetDesiredState( {4_mps, frc::Rotation2d(units::angle::degree_t(45))});

    frc::SmartDashboard::PutData("Field", &field);
}

units::volt_t Drivetrain::FindDrive_kS(units::volt_t testVoltage)
{

    
    return 0_V;
}

units::volt_t Drivetrain::FindDrive_kV(units::volt_t testVoltage)
{


    return 0_V;
}

void Drivetrain::UpdateOdometry()
{
    odometry.Update(gyro.GetRotation2d(),
                    {frontLeft.GetPosition(), frontRight.GetPosition(),
                     backLeft.GetPosition(), backRight.GetPosition()});
    field.SetRobotPose(odometry.GetEstimatedPosition());
}

void Drivetrain::UpdateTelemetry()
{
    frontLeft.UpdateTelemetry();
    frontRight.UpdateTelemetry();
    backLeft.UpdateTelemetry();
    backRight.UpdateTelemetry();

    frc::SmartDashboard::PutNumber("X Acceleration", GetXAcceleration());
    frc::SmartDashboard::PutNumber("Y Acceleration", GetYAcceleration());
}

void Drivetrain::SimMode()
{
    frontLeft.SimMode();
    frontRight.SimMode();
    backLeft.SimMode();
    backRight.SimMode();
}