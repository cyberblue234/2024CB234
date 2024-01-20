#include "Drivetrain.h"
#include "RobotExt.h"

bool fieldRelative = true;

Drivetrain::Drivetrain() : 
frontLeft(FL_DRIVE_ADDRESS, FL_SWERVE_ADDRESS, FL_CANCODER_ADDRESS, FL_OFFSET_DEGREES), 
frontRight(FR_DRIVE_ADDRESS, FR_SWERVE_ADDRESS, FR_CANCODER_ADDRESS, FR_OFFSET_DEGREES), 
backLeft(BL_DRIVE_ADDRESS, BL_SWERVE_ADDRESS, BL_CANCODER_ADDRESS, BL_OFFSET_DEGREES), 
backRight(BR_DRIVE_ADDRESS, BR_SWERVE_ADDRESS, BR_CANCODER_ADDRESS, BR_OFFSET_DEGREES), 
gyro(frc::SPI::Port::kMXP),
odometry(
    kinematics, gyro.GetRotation2d(),
    { frontLeft.GetModulePosition(), frontRight.GetModulePosition(),
    backLeft.GetModulePosition(), backRight.GetModulePosition() },
    frc::Pose2d() )
{
    gyro.Reset();
    // copies from https://pathplanner.dev/pplib-getting-started.html
    pathplanner::AutoBuilder::configureHolonomic(
        [this](){ return this->GetPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ this->ResetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return this->GetCurrentSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ this->Drive(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        Drivetrain::holonomicConfig,
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
}

void Drivetrain::Periodic() 
{
    UpdateOdometry();
}

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
        SetAnchorState();
    }
    else if (gamePad.GetRightTriggerAxis() > 0.2)
    {
        DriveWithJoystick(true);
    }
    else
    {
        DriveWithJoystick(false);
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

    frc::Rotation2d heading = gyro.GetRotation2d();
    chassisSpeeds = fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(ySpeed, xSpeed, rotation, heading)
                                                                : frc::ChassisSpeeds{ySpeed, xSpeed, rotation};

    Drive(chassisSpeeds);

    frc::SmartDashboard::PutNumber("FWD", fwd);
    frc::SmartDashboard::PutNumber("STF", stf);
    frc::SmartDashboard::PutNumber("ROT", rot);
    frc::SmartDashboard::PutNumber("XSPEED", (double)xSpeed);
    frc::SmartDashboard::PutNumber("YSPEED", (double)ySpeed);
    frc::SmartDashboard::PutNumber("ROTATION", (double)rotation);
}

// We need to change Drive() to only take in a ChassisSpeeds parameter. 

// Pass in FromRelativeSpeeds if field oriented, so path planner can use robot centric

void Drivetrain::Drive(const frc::ChassisSpeeds& speeds)
{
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