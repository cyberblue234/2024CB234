#include "Drivetrain.h"

bool fieldRelative = true;

Drivetrain::Drivetrain() : 
frontLeft(RobotMap::FL_DRIVE_ADDRESS, RobotMap::FL_SWERVE_ADDRESS, RobotMap::FL_CANCODER_ADDRESS, DrivetrainConstants::FL_OFFSET_DEGREES), 
frontRight(RobotMap::FR_DRIVE_ADDRESS, RobotMap::FR_SWERVE_ADDRESS, RobotMap::FR_CANCODER_ADDRESS, DrivetrainConstants::FR_OFFSET_DEGREES), 
backLeft(RobotMap::BL_DRIVE_ADDRESS, RobotMap::BL_SWERVE_ADDRESS, RobotMap::BL_CANCODER_ADDRESS, DrivetrainConstants::BL_OFFSET_DEGREES), 
backRight(RobotMap::BR_DRIVE_ADDRESS, RobotMap::BR_SWERVE_ADDRESS, RobotMap::BR_CANCODER_ADDRESS, DrivetrainConstants::BR_OFFSET_DEGREES), 
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
        DrivetrainConstants::holonomicConfig,
        []() {
            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this
    );
}

void Drivetrain::Periodic() 
{
    odometry.Update(gyro.GetRotation2d(), { frontLeft.GetModulePosition(), frontRight.GetModulePosition(), backLeft.GetModulePosition(), backRight.GetModulePosition() });
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
    else if (gamePad.GetRightTriggerAxis() > 0.2)
    {
        DriveWithJoystick(true);
    }
    else
    {
        DriveWithJoystick(false);
    }
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
        fwd *= DrivetrainConstants::DRIVE_SLOW_ADJUSTMENT;
    }

    double stf = gamePad.GetLeftX(); // Strafe
    stf = stf * stf * stf;
    if (abs(stf) < 0.05)
    {
        stf = 0.0;
    }
    if (limitSpeed == true)
    {
        stf *= DrivetrainConstants::DRIVE_SLOW_ADJUSTMENT;
    }

    double rot = gamePad.GetRightX(); // Rotation
    rot = rot * rot * rot;
    if (abs(rot) < 0.05)
    {
        rot = 0.0;
    }
    if (limitSpeed == true)
    {
        rot *= DrivetrainConstants::DRIVE_SLOW_ADJUSTMENT;
    }

    // Get the y speed or forward speed. Invert this because Xbox controllers return negative values when pushed forward
    auto ySpeed = units::meters_per_second_t(-fwd * DrivetrainConstants::MAX_SPEED);

    // Get the x speed or sideways/strafe speed. Needs to be inverted.
    auto xSpeed = units::meters_per_second_t(-stf * DrivetrainConstants::MAX_SPEED);

    // Get the rate of angular rotation. Needs to be inverted. Remember CCW is positive in mathematics.
    auto rotation = units::radians_per_second_t(-rot * DrivetrainConstants::MAX_ANGULAR_SPEED);

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
    auto speeds = fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(ySpeed, xSpeed, rotation, heading)
                                                                : frc::ChassisSpeeds{ySpeed, xSpeed, rotation};

    Drive(speeds);

    frc::SmartDashboard::PutNumber("FWD", fwd);
    frc::SmartDashboard::PutNumber("STF", stf);
    frc::SmartDashboard::PutNumber("ROT", rot);
    frc::SmartDashboard::PutNumber("XSPEED", (double)xSpeed);
    frc::SmartDashboard::PutNumber("YSPEED", (double)ySpeed);
    frc::SmartDashboard::PutNumber("ROTATION", (double)rotation);
}

void Drivetrain::Drive(const frc::ChassisSpeeds& speeds)
{
    chassisSpeeds = speeds;
    auto states = kinematics.ToSwerveModuleStates(speeds);

    kinematics.DesaturateWheelSpeeds(&states, DrivetrainConstants::MAX_SPEED);

    frc::SwerveModuleState fl = states[0];
    frc::SwerveModuleState fr = states[1];
    frc::SwerveModuleState bl = states[2];
    frc::SwerveModuleState br = states[3];

    frontLeft.SetDesiredState(fl, DrivetrainConstants::FL_DRIVE_ADJUSTMENT);
    frontRight.SetDesiredState(fr, DrivetrainConstants::FR_DRIVE_ADJUSTMENT);
    backLeft.SetDesiredState(bl, DrivetrainConstants::BL_DRIVE_ADJUSTMENT);
    backRight.SetDesiredState(br, DrivetrainConstants::BR_DRIVE_ADJUSTMENT);
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

    double distance = ((lcount + rcount) / 2.0) * SwerveModuleConstants::ENCODER_INCHES_PER_COUNT;

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