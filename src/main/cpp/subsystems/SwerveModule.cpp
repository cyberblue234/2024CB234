#include "subsystems/SwerveModule.h"
#include <ctre/phoenix6/configs/Configurator.hpp>

// SwerveModule constructor
SwerveModule::SwerveModule(int driveMotorChannel, int swerveMotorChannel, int canCoderChannel, double offsetDegrees)
    : driveMotor(driveMotorChannel, "rio"),
      swerveMotor(swerveMotorChannel, "rio"),
      canCoder(canCoderChannel, "rio")
{
    swerveMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration swerveMotorConfig{};

    configs::FeedbackConfigs swerveMotorFeedback{};
    swerveMotorFeedback.WithRemoteCANcoder(canCoder);
    swerveMotorConfig.WithFeedback(swerveMotorFeedback);
    
    configs::MotorOutputConfigs swerveMotorOutput{};
    swerveMotorOutput.WithInverted(signals::InvertedValue::Clockwise_Positive);
    swerveMotorConfig.WithMotorOutput(swerveMotorOutput);

    // TODO: find replacement
    //swerveMotor.ConfigVoltageCompSaturation(11.0);
    //swerveMotor.EnableVoltageCompensation(true);

    swerveMotor.GetConfigurator().Apply(swerveMotorConfig);

    // Set PID values for angle motor
    configs::SlotConfigs swervePIDFConfigs{};
    swervePIDFConfigs.kP = SwerveModuleConstants::kAngleP;
    swervePIDFConfigs.kI = SwerveModuleConstants::kAngleI;
    swervePIDFConfigs.kD = SwerveModuleConstants::kAngleD;
    swervePIDFConfigs.kV = SwerveModuleConstants::kAngleF;

    // apply gains, 50 ms total timeout
    swerveMotor.GetConfigurator().Apply(swervePIDFConfigs);

    driveMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration driveMotorConfig{};

    ResetEncoder();

    // TODO: find replacement
    // driveMotor.ConfigVoltageCompSaturation(11.0);
    // driveMotor.EnableVoltageCompensation(true);

    configs::MotorOutputConfigs driveMotorOutput{};
    driveMotorOutput.WithNeutralMode(signals::NeutralModeValue::Brake);
    driveMotorConfig.WithMotorOutput(driveMotorOutput);

    driveMotor.GetConfigurator().Apply(driveMotorConfig);

    // Set PID values for angle motor
    configs::SlotConfigs drivePIDFConfigs{};
    drivePIDFConfigs.kP = SwerveModuleConstants::kDriveP;
    drivePIDFConfigs.kI = SwerveModuleConstants::kDriveI;
    drivePIDFConfigs.kD = SwerveModuleConstants::kDriveD;
    drivePIDFConfigs.kV = SwerveModuleConstants::kDriveF;

    // apply gains, 50 ms total timeout
    driveMotor.GetConfigurator().Apply(drivePIDFConfigs);

    canCoder.GetConfigurator().Apply(configs::CANcoderConfiguration{});
    configs::CANcoderConfiguration canCoderConfig{};

    configs::MagnetSensorConfigs canCoderMagnetSensor{};
    canCoderMagnetSensor.WithMagnetOffset(offsetDegrees);

    canCoder.GetConfigurator().Apply(canCoderConfig);

    //canCoder.SetPosition(0); -> ResetCanCoder();
    //swerveMotor.SetPosition(0);
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
    currentCount = canCoder.GetPosition().GetValueAsDouble() / SwerveModuleConstants::kCancoderFeedbackCoefficient;

    // The new module position will be the the current ticks plus the change in ticks
    desiredCount = currentCount + deltaCount;
    swerveMotor.Set(desiredCount);

    // Set the drive motor to the optimized state speed

    percentSpeed = optimizedState.speed / DrivetrainConstants::MAX_SPEED;
    driveMotor.Set(percentSpeed * speedAdjustment);
}

void SwerveModule::SetDriveOpenLoopRamp(double ramp)
{
    configs::TalonFXConfiguration driveMotorConfig{};
    configs::OpenLoopRampsConfigs driveMotorOpenLoopRamps{};
    driveMotorOpenLoopRamps.WithDutyCycleOpenLoopRampPeriod(ramp);
    driveMotorConfig.WithOpenLoopRamps(driveMotorOpenLoopRamps);
    driveMotor.GetConfigurator().Apply(driveMotorConfig);
}