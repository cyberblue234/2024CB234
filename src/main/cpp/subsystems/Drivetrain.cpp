#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain(Limelight *limelight3) : frontLeft(RobotMap::FL_DRIVE_ADDRESS, RobotMap::FL_SWERVE_ADDRESS, RobotMap::FL_CANCODER_ADDRESS, DrivetrainConstants::FL_OFFSET_DEGREES),
                                                frontRight(RobotMap::FR_DRIVE_ADDRESS, RobotMap::FR_SWERVE_ADDRESS, RobotMap::FR_CANCODER_ADDRESS, DrivetrainConstants::FR_OFFSET_DEGREES),
                                                backLeft(RobotMap::BL_DRIVE_ADDRESS, RobotMap::BL_SWERVE_ADDRESS, RobotMap::BL_CANCODER_ADDRESS, DrivetrainConstants::BL_OFFSET_DEGREES),
                                                backRight(RobotMap::BR_DRIVE_ADDRESS, RobotMap::BR_SWERVE_ADDRESS, RobotMap::BR_CANCODER_ADDRESS, DrivetrainConstants::BR_OFFSET_DEGREES),
                                                gyro(frc::SPI::Port::kMXP),
                                                odometry(
                                                    kinematics, gyro.GetRotation2d(),
                                                    {frontLeft.GetModulePosition(), frontRight.GetModulePosition(),
                                                     backLeft.GetModulePosition(), backRight.GetModulePosition()},
                                                    frc::Pose2d())
{
    gyro.Reset();
    // copies from https://pathplanner.dev/pplib-getting-started.html
    pathplanner::AutoBuilder::configureHolonomic(
        [this]()
        { return this->GetPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose)
        { this->ResetPose(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this]()
        { return this->GetCurrentSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds)
        { this->Drive(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        DrivetrainConstants::holonomicConfig,
        []()
        {
            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance)
            {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this);

    this->limelight3 = limelight3;

    rotationController.SetTolerance(5.0);

    pathplanner::PathPlannerLogging::setLogActivePathCallback([this](auto poses) {
        this->field.GetObject("path")->SetPoses(poses);
    });

    frc::SmartDashboard::PutData("Field", &field);
}

void Drivetrain::Periodic()
{
    odometry.Update(gyro.GetRotation2d(), {frontLeft.GetModulePosition(), frontRight.GetModulePosition(), backLeft.GetModulePosition(), backRight.GetModulePosition()});
    if (cycle % 5 == 0)
    {
        limelight3->UpdateLimelightTracking();
        limelight3->UpdateTelemetry();
        if (limelight3->GetTargetValid() == 1 && abs((double)limelight3->GetRobotPose().X() - (double)odometry.GetEstimatedPosition().X()) < 1)
            odometry.AddVisionMeasurement(limelight3->GetRobotPose(), frc::Timer::GetFPGATimestamp());
    }
    if (cycle < 10)
    {
        limelight3->UpdateLimelightTracking();
        ResetPose(limelight3->GetRobotPose());
    }
    cycle++;

    field.SetRobotPose(GetPose());

    UpdateTelemetry();
}

void Drivetrain::DriveWithInput(double fwd, double stf, double rot, bool limitSpeed)
{
    fwd = fwd * fwd * fwd;
    if (abs(fwd) < 0.05)
    {
        fwd = 0.0;
    }
    if (limitSpeed == true)
    {
        fwd *= DrivetrainConstants::DRIVE_SLOW_ADJUSTMENT;
    }

    stf = stf * stf * stf;
    if (abs(stf) < 0.05)
    {
        stf = 0.0;
    }
    if (limitSpeed == true)
    {
        stf *= DrivetrainConstants::DRIVE_SLOW_ADJUSTMENT;
    }

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

    frc::Rotation2d heading = gyro.GetRotation2d(); //odometry.GetEstimatedPosition().Rotation().RotateBy(frc::Rotation2d(units::angle::degree_t(180)));
    auto speeds = fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(ySpeed, xSpeed, rotation, heading)
                                : frc::ChassisSpeeds{ySpeed, xSpeed, rotation};

    Drive(speeds);

    frc::SmartDashboard::PutNumber("Y Speed", (double)ySpeed);
    frc::SmartDashboard::PutNumber("X Speed", (double)xSpeed);
    frc::SmartDashboard::PutNumber("Rot", (double)rot);
}

void Drivetrain::Drive(const frc::ChassisSpeeds &speeds)
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

double Drivetrain::RotationControl(double rotInput, bool alignToAprilTag)
{
    if (alignToAprilTag)
    {
        limelight3->SetPipelineID(Limelight::kSpeakerDetection);
        rotInput = -rotationController.Calculate(limelight3->GetAprilTagOffset(), 0);
        return rotInput;
    }
    else
    {
        limelight3->SetPipelineID(Limelight::kAprilTag);
        rotInput = rotInput * rotInput * rotInput;
        lastGyroYaw = (double)gyro.GetRotation2d().Degrees();
        return rotInput;
    }
}

void Drivetrain::AlignToSpeaker()
{
    DriveWithInput(0.0, 0.0, RotationControl(0, true), false);    
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

void Drivetrain::UpdateTelemetry()
{
    frc::SmartDashboard::PutNumber("Odometry X", (double)GetPose().X());
    frc::SmartDashboard::PutNumber("Odometry Y", (double)GetPose().Y());
    frc::SmartDashboard::PutNumber("Odometry Rot", (double)GetPose().Rotation().Degrees());

    frc::SmartDashboard::PutNumber("FL Drive Rotations", frontLeft.GetDriveEncoder());

}

void Drivetrain::ResetCancoders()
{
    frontLeft.ResetCanCoder();
    frontRight.ResetCanCoder();
    backLeft.ResetCanCoder();
    backRight.ResetCanCoder();
}

double Drivetrain::GetDriveDistance()
{
    double lcount = abs(frontLeft.GetDriveEncoder());
    double rcount = abs(backRight.GetDriveEncoder());

    double distance = ((lcount + rcount) / 2.0) * SwerveModuleConstants::ENCODER_INCHES_PER_COUNT;
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
    gyro.Reset();
};

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