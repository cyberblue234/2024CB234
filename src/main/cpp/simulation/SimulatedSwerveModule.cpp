#include "subsystems\simulation\SimulatedSwerveModule.h"

SimulatedSwerveModule::SimulatedSwerveModule(int driveMotorID, int turnMotorID, int canCoderID, double canCoderMagnetOffset)
    : driveMotor(driveMotorID, "rio"),
      turnMotor(turnMotorID, "rio"),
      canCoder(canCoderID, "rio")
{
    turnMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration turnMotorConfig{};

    configs::FeedbackConfigs turnMotorFeedback{};
    turnMotorFeedback.WithRemoteCANcoder(canCoder);

    turnMotorConfig.WithFeedback(turnMotorFeedback);

    configs::MotorOutputConfigs turnMotorOutput{};
    turnMotorOutput.WithInverted(signals::InvertedValue::Clockwise_Positive);
    turnMotorConfig.WithMotorOutput(turnMotorOutput);

    configs::CurrentLimitsConfigs swerveCurrentLimit{};
    swerveCurrentLimit.WithStatorCurrentLimit(120);
    swerveCurrentLimit.WithStatorCurrentLimitEnable(true);
    turnMotorConfig.WithCurrentLimits(swerveCurrentLimit);

    configs::OpenLoopRampsConfigs turnMotorOpenLoopRamps{};
    turnMotorOpenLoopRamps.WithDutyCycleOpenLoopRampPeriod(0.15);
    turnMotorConfig.WithOpenLoopRamps(turnMotorOpenLoopRamps);

    // TODO: find replacement -- try configs::MotorOutputConfigs::WithPeakForwardDutyCycle and configs::MotorOutputConfigs::WithPeakReverseDutyCycle
    // turnMotor.ConfigVoltageCompSaturation(11.0);
    // turnMotor.EnableVoltageCompensation(true);

    turnMotor.GetConfigurator().Apply(turnMotorConfig);

    driveMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration driveMotorConfig{};

    SetEncoder(0);

    // TODO: find replacement
    // driveMotor.ConfigVoltageCompSaturation(11.0);
    // driveMotor.EnableVoltageCompensation(true);

    configs::MotorOutputConfigs driveMotorOutput{};
    driveMotorOutput.WithNeutralMode(signals::NeutralModeValue::Brake);
    driveMotorConfig.WithMotorOutput(driveMotorOutput);

    configs::CurrentLimitsConfigs driveCurrentLimit{};
    driveCurrentLimit.WithStatorCurrentLimit(120);
    driveCurrentLimit.WithStatorCurrentLimitEnable(true);
    driveMotorConfig.WithCurrentLimits(driveCurrentLimit);

    configs::OpenLoopRampsConfigs driveMotorOpenLoopRamps{};
    driveMotorOpenLoopRamps.WithDutyCycleOpenLoopRampPeriod(0.15);
    driveMotorConfig.WithOpenLoopRamps(driveMotorOpenLoopRamps);

    driveMotor.GetConfigurator().Apply(driveMotorConfig);

    canCoder.GetConfigurator().Apply(configs::CANcoderConfiguration{});
    configs::CANcoderConfiguration canCoderConfig{};

    configs::MagnetSensorConfigs canCoderMagnetSensor{};
    canCoderMagnetSensor.WithMagnetOffset(canCoderMagnetOffset);
    canCoderMagnetSensor.WithAbsoluteSensorRange(signals::AbsoluteSensorRangeValue::Unsigned_0To1);

    canCoderConfig.WithMagnetSensor(canCoderMagnetSensor);
    canCoder.GetConfigurator().Apply(canCoderConfig);
    ResetCanCoder(0);

    turnPIDController.EnableContinuousInput(-units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});
}

void SimulatedSwerveModule::SetDesiredState(const frc::SwerveModuleState &referenceState)
{
    frc::SmartDashboard::PutNumber(std::to_string(driveMotor.GetDeviceID()) + " encoderDistance", GetPosition().distance.value());
    frc::SmartDashboard::PutNumber(std::to_string(driveMotor.GetDeviceID()) + " canCoderDistance", GetPosition().angle.Degrees().value());
    frc::SmartDashboard::PutNumber(std::to_string(driveMotor.GetDeviceID()) + " canCoderDistance2", GetCanCoderDistance().value());
    frc::SmartDashboard::PutNumber(std::to_string(driveMotor.GetDeviceID()) + " encoderVelocity", (double)GetState().speed);
    

    frc::Rotation2d encoderRotation{units::radian_t{GetCanCoderDistance()}};

    // Optimize the reference state to avoid spinning further than 90 degrees
    auto state = frc::SwerveModuleState::Optimize(referenceState, encoderRotation);

    frc::SmartDashboard::PutNumber(std::to_string(driveMotor.GetDeviceID()) + " setEncoderVelocity", state.speed.value());
    frc::SmartDashboard::PutNumber(std::to_string(driveMotor.GetDeviceID()) + " setCancoderDistance", state.angle.Degrees().value());
    // Scale speed by cosine of angle error. This scales down movement
    // perpendicular to the desired direction of travel that can occur when
    // modules change directions. This results in smoother driving.
    state.speed *= (state.angle - encoderRotation).Cos();

    // Calculate the drive output from the drive PID controller.
    const auto driveOutput = drivePIDController.Calculate(GetDriveVelocity().value(), state.speed.value());

    const auto driveFeedforwardValue = driveFeedforward.Calculate(state.speed);

    // Calculate the turning motor output from the turning PID controller.
    const auto turnOutput = turnPIDController.Calculate(units::radian_t{GetCanCoderDistance()}, state.angle.Radians());

    const auto turnFeedforwardValue = turnFeedforward.Calculate(turnPIDController.GetSetpoint().velocity);

    // Set the motor outputs.
    driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforwardValue);
    turnMotor.SetVoltage(units::volt_t{turnOutput} + turnFeedforwardValue);

    frc::SmartDashboard::PutNumber(std::to_string(driveMotor.GetDeviceID()) + " driveOutput", driveOutput);
    frc::SmartDashboard::PutNumber(std::to_string(driveMotor.GetDeviceID()) + " turnOutput", turnOutput);
    frc::SmartDashboard::PutNumber(std::to_string(driveMotor.GetDeviceID()) + " driveFeedforward", (double)driveFeedforwardValue);
    frc::SmartDashboard::PutNumber(std::to_string(driveMotor.GetDeviceID()) + " turnFeedForward", (double)turnFeedforwardValue);

    sim::TalonFXSimState& driveMotorSim = driveMotor.GetSimState();
    sim::TalonFXSimState& turnMotorSim = turnMotor.GetSimState();
    sim::CANcoderSimState& canCoderSim = canCoder.GetSimState();

   // set the supply voltage of the TalonFX
   driveMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
   turnMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
   canCoderSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

   // get the motor voltage of the TalonFX
   auto driveMotorVoltage = driveMotorSim.GetMotorVoltage();
   auto turnMotorVoltage = turnMotorSim.GetMotorVoltage();

   // use the motor voltage to calculate new position and velocity
   // using WPILib's DCMotorSim class for physics simulation
   driveMotorSimModel.SetInputVoltage(driveMotorVoltage);
   driveMotorSimModel.Update(20_ms); // assume 20 ms loop time
   turnMotorSimModel.SetInputVoltage(turnMotorVoltage);
   turnMotorSimModel.Update(20_ms); // assume 20 ms loop time

   // apply the new rotor position and velocity to the TalonFX;
   // note that this is rotor position/velocity (before gear ratio), but
   // DCMotorSim returns mechanism position/velocity (after gear ratio)
   driveMotorSim.SetRawRotorPosition(kDriveGearRatio * driveMotorSimModel.GetAngularPosition());
   driveMotorSim.SetRotorVelocity(kDriveGearRatio * driveMotorSimModel.GetAngularVelocity());
   turnMotorSim.SetRawRotorPosition(turnMotorSimModel.GetAngularPosition());
   canCoderSim.SetRawPosition(turnMotorSimModel.GetAngularPosition() / 360 * 2 * std::numbers::pi);
   turnMotorSim.SetRotorVelocity(turnMotorSimModel.GetAngularVelocity());
   canCoderSim.SetVelocity(turnMotorSimModel.GetAngularVelocity());
}
