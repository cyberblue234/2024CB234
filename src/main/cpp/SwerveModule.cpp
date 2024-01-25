#include "SwerveModule.h"

// SwerveModule constructor
SwerveModule::SwerveModule(int driveMotorChannel, int swerveMotorChannel, int canCoderChannel, double offsetDegrees)
    : driveMotor(driveMotorChannel, "rio"),
      swerveMotor(swerveMotorChannel, "rio"),
      canCoder(canCoderChannel, "rio")
{
    swerveMotor.ConfigFactoryDefault();
    // Select the canCoder to assign to RemoteSensor0
    swerveMotor.ConfigRemoteFeedbackFilter(canCoder, 0);
    // Select RemoteSensor0 (canCoder) as the selected feedback sensor
    swerveMotor.ConfigSelectedFeedbackSensor(RemoteFeedbackDevice::RemoteFeedbackDevice_RemoteSensor0);
    // Set the sensor phase
    swerveMotor.SetSensorPhase(true);
    swerveMotor.SetInverted(true);
    // Config max voltage to motor
    swerveMotor.ConfigVoltageCompSaturation(11.0);
    swerveMotor.EnableVoltageCompensation(true);

    // Set PID values for angle motor
    swerveMotor.Config_kP(0, SwerveModuleConstants::kAngleP);
    swerveMotor.Config_kI(0, SwerveModuleConstants::kAngleI);
    swerveMotor.Config_kD(0, SwerveModuleConstants::kAngleD);
    swerveMotor.Config_kF(0, SwerveModuleConstants::kAngleF);

    driveMotor.ConfigFactoryDefault();
    driveMotor.SetSelectedSensorPosition(0);
    driveMotor.ConfigVoltageCompSaturation(11.0);
    driveMotor.EnableVoltageCompensation(true);
    driveMotor.SetNeutralMode(NeutralMode::Brake);

    driveMotor.Config_kP(0, SwerveModuleConstants::kDriveP);
    driveMotor.Config_kI(0, SwerveModuleConstants::kDriveI);
    driveMotor.Config_kD(0, SwerveModuleConstants::kDriveD);
    driveMotor.Config_kF(0, SwerveModuleConstants::kDriveF);

    canCoder.ConfigFactoryDefault();
    canCoder.ConfigMagnetOffset(offsetDegrees);
    canCoder.SetPosition(0);
    swerveMotor.SetSelectedSensorPosition(0);
}

// Set the speed + rotation of the swerve module from a SwerveModuleState object
// param: desiredState - A SwerveModuleState representing the desired new state of the module
void SwerveModule::SetDesiredState(const frc::SwerveModuleState desiredState, double speedAdjustment)
{
    currentAngle = GetAngle(); // 0 - 360 degrees
    // SwerveModuleState contains information about the velocity and angle of a swerve module
    // Optimize to avoid spinning more than 90 degrees, or pi/2 radians
    frc::SwerveModuleState optimizedState = frc::SwerveModuleState::Optimize(desiredState, currentAngle);

    // Find the difference between our current rotational position and our new rotational position
    deltaAngle = optimizedState.angle.operator-(currentAngle);

    // Find how much to turn the module in CANCoder ticks
    deltaCount = ((double)deltaAngle.Degrees() / 360.0) * SwerveModuleConstants::kCancoderCountsPerRotation;

    // Get the current position of the module in CANCoder ticks
    // Divide by the feedback coefficient to convert from degrees to ticks
    // GetPosition defaults to return degrees.
    currentCount = canCoder.GetPosition() / SwerveModuleConstants::kCancoderFeedbackCoefficient;

    // The new module position will be the the current ticks plus the change in ticks
    desiredCount = currentCount + deltaCount;
    swerveMotor.Set(TalonFXControlMode::Position, desiredCount);

    // Set the drive motor to the optimized state speed

    percentSpeed = optimizedState.speed / DrivetrainConstants::MAX_SPEED;
    driveMotor.Set(TalonFXControlMode::PercentOutput, percentSpeed * speedAdjustment);
}