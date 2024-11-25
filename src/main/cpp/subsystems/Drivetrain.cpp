#include "subsystems/Drivetrain.h"

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative,
                       units::second_t period)
{
    frc::SmartDashboard::PutNumber("xSpeed", xSpeed.value());
    frc::SmartDashboard::PutNumber("ySpeed", ySpeed.value());
    frc::SmartDashboard::PutNumber("rot", rot.value());
    frc::SmartDashboard::PutNumber("period", period.value());
    
    auto states =
        kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
            fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                                xSpeed, ySpeed, rot, gyro.GetRotation2d())
                          : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
            period));

    kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

    auto [fl, fr, bl, br] = states;

    frc::SmartDashboard::PutNumber("frontLeftDesiredState.speed", fl.speed.value());
    frc::SmartDashboard::PutNumber("frontLeftDesiredState.angle", fl.angle.Radians().value());
    frc::SmartDashboard::PutNumber("frontRightDesiredState.speed", fr.speed.value());
    frc::SmartDashboard::PutNumber("frontRightDesiredState.angle", fr.angle.Radians().value());
    frc::SmartDashboard::PutNumber("backLeftDesiredState.speed", bl.speed.value());
    frc::SmartDashboard::PutNumber("backLeftDesiredState.angle", bl.angle.Radians().value());
    frc::SmartDashboard::PutNumber("backRightDesiredState.speed", br.speed.value());
    frc::SmartDashboard::PutNumber("backRightDesiredState.angle", br.angle.Radians().value());

    frontLeft.SetDesiredState(fl);
    frontRight.SetDesiredState(fr);
    backLeft.SetDesiredState(bl);
    backRight.SetDesiredState(br);
}

void Drivetrain::UpdateOdometry()
{
    odometry.Update(gyro.GetRotation2d(),
                    {frontLeft.GetPosition(), frontRight.GetPosition(),
                     backLeft.GetPosition(), backRight.GetPosition()});
}

void Drivetrain::UpdateTelemetry()
{
    
}