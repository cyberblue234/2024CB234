
#include "RobotExt.h"
#include "SwerveModule.h"
#include "Drivetrain.h"

#include <frc/geometry/Rotation2d.h>
#include <numbers>

bool fieldRelative = true;

void Drivetrain::DriveControl()
{
    alignmentOn = frc::SmartDashboard::GetBoolean("ALIGNMENT_ON", false);
    frc::SmartDashboard::PutBoolean("ALIGNMENT_ON", alignmentOn);
    frc::SmartDashboard::PutBoolean("ALIGNMENT_LED", alignmentOn);

    if (alignmentOn)
    {
        AlignSwerveDrive();
    }
    else if (controls.GetRawButton(DRIVE_ANCHOR_SWITCH) == true)
    {
        swerve.SetAnchorState();
    }
    else if (gamePad.GetRightTriggerAxis() > 0.2)
    {
        swerve.DriveWithJoystick(true);
    }
    else
    {
        swerve.DriveWithJoystick(false);
    }

    double pitch = GetGyroPitch();
    if (pitch > 2.5 || pitch < -2.5)
    {
        frc::SmartDashboard::PutBoolean("PITCH_LED", true);
    }
    else
    {
        frc::SmartDashboard::PutBoolean("PITCH_LED", false);
    }

    frc::SmartDashboard::PutNumber("GYRO_PITCH", GetGyroPitch());
    frc::SmartDashboard::PutNumber("GYRO_ROLL", GetGyroRoll());
    frc::SmartDashboard::PutNumber("FL_ANGLE", frontLeft.GetSwervePosition());
    frc::SmartDashboard::PutNumber("FR_ANGLE", frontRight.GetSwervePosition());
    frc::SmartDashboard::PutNumber("BL_ANGLE", backLeft.GetSwervePosition());
    frc::SmartDashboard::PutNumber("BR_ANGLE", backRight.GetSwervePosition());

    frc::SmartDashboard::PutNumber("Odometry X", double(odometry.GetPose().X()));
    frc::SmartDashboard::PutNumber("Odometry Y", double(odometry.GetPose().Y()));
    frc::SmartDashboard::PutNumber("Odometry Angle", double(odometry.GetPose().Rotation().Degrees()));
}

void Drivetrain::DriveWithJoystick(bool limitSpeed)
{
    double fwd = gamePad.GetLeftY(); // Forward
    fwd = fwd * fwd * fwd;
    if (abs(fwd) < 0.05)
    {
        fwd = 0.0;
    }
    if (limitSpeed == true)
    {
        fwd *= DRIVE_SLOW_ADJUSTMENT;
    }

    double stf = gamePad.GetLeftX(); // Strafe
    stf = stf * stf * stf;
    if (abs(stf) < 0.05)
    {
        stf = 0.0;
    }
    if (limitSpeed == true)
    {
        stf *= DRIVE_SLOW_ADJUSTMENT;
    }

    double rot = gamePad.GetRightX(); // Rotation
    rot = rot * rot * rot;
    if (abs(rot) < 0.05)
    {
        rot = 0.0;
    }
    if (limitSpeed == true)
    {
        rot *= DRIVE_SLOW_ADJUSTMENT;
    }

    // Get the y speed or forward speed. Invert this because Xbox controllers return negative values when pushed forward
    auto ySpeed = units::meters_per_second_t(-fwd * Drivetrain::MAX_SPEED);

    // Get the x speed or sideways/strafe speed. Needs to be inverted.
    auto xSpeed = units::meters_per_second_t(-stf * Drivetrain::MAX_SPEED);

    // Get the rate of angular rotation. Needs to be inverted. Remember CCW is positive in mathematics.
    auto rotation = units::radians_per_second_t(-rot * Drivetrain::MAX_ANGULAR_SPEED);

    if (gamePad.GetXButton() == true)
    {
        fieldRelative = true;
    }
    if (gamePad.GetBButton() == true)
    {
        fieldRelative = false;
    }
    if (gamePad.GetYButton() == true)
    {
        ResetGyroAngle();
    }

    Drive(xSpeed, ySpeed, rotation, fieldRelative);

    GetDriveDistance();

    frc::SmartDashboard::PutNumber("FWD", fwd);
    frc::SmartDashboard::PutNumber("STF", stf);
    frc::SmartDashboard::PutNumber("ROT", rot);
    frc::SmartDashboard::PutNumber("XSPEED", (double)xSpeed);
    frc::SmartDashboard::PutNumber("YSPEED", (double)ySpeed);
    frc::SmartDashboard::PutNumber("ROTATION", (double)rotation);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rotation, bool fieldRelative)
{
    frc::Rotation2d heading = gyro.GetRotation2d();
    chassisSpeeds = fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(ySpeed, xSpeed, rotation, heading)
                                                                : frc::ChassisSpeeds{ySpeed, xSpeed, rotation};
    auto states = kinematics.ToSwerveModuleStates(chassisSpeeds);

    kinematics.DesaturateWheelSpeeds(&states, MAX_SPEED);

    // each state is the angle and velocity of each module
    frc::SwerveModuleState fl = states[0];
    frc::SwerveModuleState fr = states[1];
    frc::SwerveModuleState bl = states[2];
    frc::SwerveModuleState br = states[3];

    frontLeft.SetDesiredState(fl, FL_DRIVE_ADJUSTMENT);
    frontRight.SetDesiredState(fr, FR_DRIVE_ADJUSTMENT);
    backLeft.SetDesiredState(bl, BL_DRIVE_ADJUSTMENT);
    backRight.SetDesiredState(br, BR_DRIVE_ADJUSTMENT);

    frc::SmartDashboard::PutNumber("FLSPEED", frontLeft.GetDriveRPM() * 0.002234); 
    frc::SmartDashboard::PutNumber("FRSPEED", frontRight.GetDriveRPM() * 0.002234);
    frc::SmartDashboard::PutNumber("BLSPEED", backLeft.GetDriveRPM() * 0.002234); 
    frc::SmartDashboard::PutNumber("BRSPEED", backRight.GetDriveRPM() * 0.002234);
    frc::SmartDashboard::PutNumber("GYRO", gyro.GetAngle());
    frc::SmartDashboard::PutNumber("HEADING", (double)heading.Degrees());
    frc::SmartDashboard::PutNumber("FL_RPM", frontLeft.GetDriveRPM());
}

frc::Pose2d Drivetrain::UpdateOdometry() 
{
    return odometry.Update(gyro.GetRotation2d(),
    {
      frontLeft.GetModulePosition(), frontRight.GetModulePosition(),
      backLeft.GetModulePosition(), backRight.GetModulePosition()
    }
  );
}

void Drivetrain::SetAnchorState()
{
    frc::SwerveModuleState fl = {0_mps, frc::Rotation2d(units::angle::degree_t(45))};
    frc::SwerveModuleState fr = {0_mps, frc::Rotation2d(units::angle::degree_t(-45))};
    frc::SwerveModuleState bl = {0_mps, frc::Rotation2d(units::angle::degree_t(-45))};
    frc::SwerveModuleState br = {0_mps, frc::Rotation2d(units::angle::degree_t(45))};

    frontLeft.SetDesiredState(fl, 0.0);
    frontRight.SetDesiredState(fr, 0.0);
    backLeft.SetDesiredState(bl, 0.0);
    backRight.SetDesiredState(br, 0.0);
}

void Drivetrain::DriveAtAngle(double speed, double angle)
{
    frc::SwerveModuleState fl = {units::velocity::meters_per_second_t(speed), frc::Rotation2d(units::angle::degree_t(angle))};
    frc::SwerveModuleState fr = {units::velocity::meters_per_second_t(speed), frc::Rotation2d(units::angle::degree_t(angle))};
    frc::SwerveModuleState bl = {units::velocity::meters_per_second_t(speed), frc::Rotation2d(units::angle::degree_t(angle))};
    frc::SwerveModuleState br = {units::velocity::meters_per_second_t(speed), frc::Rotation2d(units::angle::degree_t(angle))};

    frontLeft.SetDesiredState(fl, FL_DRIVE_ADJUSTMENT);
    frontRight.SetDesiredState(fr, FR_DRIVE_ADJUSTMENT);
    backLeft.SetDesiredState(bl, BL_DRIVE_ADJUSTMENT);
    backRight.SetDesiredState(br, BR_DRIVE_ADJUSTMENT);
}

void Drivetrain::SetDriveStop()
{
    frc::SwerveModuleState fl = {0_mps, frc::Rotation2d(units::angle::degree_t(0.0))};
    frc::SwerveModuleState fr = {0_mps, frc::Rotation2d(units::angle::degree_t(0.0))};
    frc::SwerveModuleState bl = {0_mps, frc::Rotation2d(units::angle::degree_t(0.0))};
    frc::SwerveModuleState br = {0_mps, frc::Rotation2d(units::angle::degree_t(0.0))};

    frontLeft.SetDesiredState(fl, 0.0);
    frontRight.SetDesiredState(fr, 0.0);
    backLeft.SetDesiredState(bl, 0.0);
    backRight.SetDesiredState(br, 0.0);
}

void Drivetrain::DriveSidewaysSlow()
{
    double stf = pow(gamePad.GetLeftX(), 1) * 0.2; // Strafe

    if (abs(stf) < 0.05)
    {
        stf = 0.0;
    }

    frc::SwerveModuleState fl = {units::meters_per_second_t(-stf * Drivetrain::MAX_SPEED), frc::Rotation2d(units::angle::degree_t(90))};
    frc::SwerveModuleState fr = {units::meters_per_second_t(-stf * Drivetrain::MAX_SPEED), frc::Rotation2d(units::angle::degree_t(90))};
    frc::SwerveModuleState bl = {units::meters_per_second_t(-stf * Drivetrain::MAX_SPEED), frc::Rotation2d(units::angle::degree_t(90))};
    frc::SwerveModuleState br = {units::meters_per_second_t(-stf * Drivetrain::MAX_SPEED), frc::Rotation2d(units::angle::degree_t(90))};

    frontLeft.SetDesiredState(fl, FL_DRIVE_ADJUSTMENT);
    frontRight.SetDesiredState(fr, FR_DRIVE_ADJUSTMENT);
    backLeft.SetDesiredState(bl, BL_DRIVE_ADJUSTMENT);
    backRight.SetDesiredState(br, BR_DRIVE_ADJUSTMENT);
}

void Drivetrain::SetDriveOpenLoopRamp(double ramp)
{
    frontLeft.SetDriveOpenLoopRamp(ramp);
    frontRight.SetDriveOpenLoopRamp(ramp);
    backLeft.SetDriveOpenLoopRamp(ramp);
    backRight.SetDriveOpenLoopRamp(ramp);
}

void Drivetrain::ResetCancoders()
{
    frontLeft.ResetCancoder();
    frontRight.ResetCancoder();
    backLeft.ResetCancoder();
    backRight.ResetCancoder();
}

double Drivetrain::GetDriveDistance()
{
    double lcount = abs(frontLeft.GetDriveEncoder());
    double rcount = abs(backRight.GetDriveEncoder());

    double distance = ((lcount + rcount) / 2.0) * SwerveModule::ENCODER_INCHES_PER_COUNT;

    frc::SmartDashboard::PutNumber("FLCOUNT", lcount);
    frc::SmartDashboard::PutNumber("FRCOUNT", frontRight.GetDriveEncoder());
    frc::SmartDashboard::PutNumber("BLCOUNT", backLeft.GetDriveEncoder());
    frc::SmartDashboard::PutNumber("BRCOUNT", backRight.GetDriveEncoder());
    frc::SmartDashboard::PutNumber("DISTANCE", distance);
    return distance;
}

void Drivetrain::ResetDriveEncoders()
{
    frontLeft.ResetEncoder();
    frontRight.ResetEncoder();
    backLeft.ResetEncoder();
    backRight.ResetEncoder();
}

void Drivetrain::ResetGyroAngle()
{
    if (gyro_reset_reversed == true)
    {
        gyro.SetAngleAdjustment(gyro.GetAngle() - 180.0);
        gyro_reset_reversed = false;
    }
    gyro.Reset();
};

void Drivetrain::ResetGyroForAuto()
{
    gyro.Reset();
    gyro_reset_reversed = true;
}

void Drivetrain::AlignSwerveDrive()
{
    double speed = 0.0;

    frontLeft.SetDriveMotor(speed);
    frontRight.SetDriveMotor(speed);
    backLeft.SetDriveMotor(speed);
    backRight.SetDriveMotor(speed);
    frontLeft.SetSwerveMotor(speed);
    frontRight.SetSwerveMotor(speed);
    backLeft.SetSwerveMotor(speed);
    backRight.SetSwerveMotor(speed);
}