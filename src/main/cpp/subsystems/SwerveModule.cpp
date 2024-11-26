#include "subsystems/SwerveModule.h"

SwerveModule::SwerveModule(std::string name, int driveMotorID, int turnMotorID, int canCoderID, double canCoderMagnetOffset)
    : driveMotor(driveMotorID, "rio"),
      turnMotor(turnMotorID, "rio"),
      canCoder(canCoderID, "rio")
{
    this->name = name;
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

    // Set PID values for angle motor
    configs::SlotConfigs drivePIDConfigs{};
    drivePIDFConfigs.kP = kDriveP;
    drivePIDFConfigs.kI = kDriveI;
    drivePIDFConfigs.kD = kDriveD;
    driveMotor.GetConfigurator().Apply(drivePIDConfigs);

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

    // Set PID values for angle motor
    configs::SlotConfturnPIDConfigs{};
    turnPIDFConfigs.kP = kTurnP;
    turnPIDFConfigs.kI = kTurnI;
    turnPIDFConfigs.kD = kTurnD;
    turnMotor.GetConfigurator().Apply(turnPIDConfigs);

    canCoder.GetConfigurator().Apply(configs::CANcoderConfiguration{});
    configs::CANcoderConfiguration canCoderConfig{};

    configs::MagnetSensorConfigs canCoderMagnetSensor{};
    canCoderMagnetSensor.WithMagnetOffset(canCoderMagnetOffset);
    canCoderMagnetSensor.WithAbsoluteSensorRange(signals::AbsoluteSensorRangeValue::Unsigned_0To1);
    canCoderMagnetSensor.WithSensorDirection(signals::SensorDirectionValue::Clockwise_Positive);

    canCoderConfig.WithMagnetSensor(canCoderMagnetSensor);
    canCoder.GetConfigurator().Apply(canCoderConfig);
    SetCanCoder(0);
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState &referenceState)
{
    // Optimize the reference state to avoid spinning further than 90 degrees
    auto state = frc::SwerveModuleState::Optimize(referenceState, GetAngle());

    // Scale speed by cosine of angle error. This scales down movement
    // perpendicular to the desired direction of travel that can occur when
    // modules change directions. This results in smoother driving.
    state.speed *= (state.angle - GetAngle()).Cos();

    // Calculate the turning motor output from the turning PID controller.
    // const auto turnOutput = turnPIDController.Calculate(units::radian_t{GetCanCoderDistance()}, state.angle.Radians());
    auto& turnPos = turnPositionOut.WithPosition(state.angle.Degrees().value() / 360);
    const auto turnFeedforwardValue = turnFeedforward.Calculate(turnPos.Velocity);
    turnMotor.SetControl(turnPos.WithFeedForward(turnFeedforwardValue));
    // Set the motor outputs.
    auto& driveVelocity = driveVelocityOut.WithVelocity(units::angular_velocity::turns_per_second_t(state.speed.value() / kDriveDistanceRatio));
    const auto turnFeedforwardValue = driveFeedforward.Calculate(state.speed);
    driveMotor.SetControl(driveVelocity.WithFeedForward(turnFeedforwardValue));

    frc::SmartDashboard::PutNumber(name + " SetSpeed", state.speed.value());
    frc::SmartDashboard::PutNumber(name + " SetAngle", state.angle.Degrees().value());
    frc::SmartDashboard::PutNumber(name + " Drive FF", driveFeedforwardValue.value());
    frc::SmartDashboard::PutNumber(name + " Turn FF", turnFeedforwardValue.value());

    
    // sim::TalonFXSimState& driveMotorSim = driveMotor.GetSimState();
    // sim::TalonFXSimState& turnMotorSim = turnMotor.GetSimState();
    // sim::CANcoderSimState& canCoderSim = canCoder.GetSimState();

    // // set the supply voltage of the TalonFX
    // driveMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    // turnMotorSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
    // canCoderSim.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    // // get the motor voltage of the TalonFX
    // auto driveMotorVoltage = driveMotorSim.GetMotorVoltage();
    // auto turnMotorVoltage = turnMotorSim.GetMotorVoltage();

    // // use the motor voltage to calculate new position and velocity
    // // using WPILib's DCMotorSim class for physics simulation
    // driveMotorSimModel.SetInputVoltage(driveMotorVoltage);
    // driveMotorSimModel.Update(20_ms); // assume 20 ms loop time
    // turnMotorSimModel.SetInputVoltage(turnMotorVoltage);
    // turnMotorSimModel.Update(20_ms); // assume 20 ms loop time

    // // apply the new rotor position and velocity to the TalonFX;
    // // note that this is rotor position/velocity (before gear ratio), but
    // // DCMotorSim returns mechanism position/velocity (after gear ratio)
    // driveMotorSim.SetRawRotorPosition(kDriveGearRatio * driveMotorSimModel.GetAngularPosition());
    // driveMotorSim.SetRotorVelocity(kDriveGearRatio * driveMotorSimModel.GetAngularVelocity());
    // turnMotorSim.SetRawRotorPosition(kTurnGearRatio * turnMotorSimModel.GetAngularPosition());
    // double canCoderPosInt;
    // double canCoderPos = std::modf(turnMotorSimModel.GetAngularPosition().value(), &canCoderPosInt);
    // if (canCoderPos < 0) canCoderPos += 1.0;
    // canCoderSim.SetRawPosition(units::angle::turn_t(canCoderPos));
    // turnMotorSim.SetRotorVelocity(kTurnGearRatio * turnMotorSimModel.GetAngularVelocity());
    // canCoderSim.SetVelocity(turnMotorSimModel.GetAngularVelocity());
}

void SwerveModule::UpdateTelemetry()
{
    frc::SmartDashboard::PutNumber(name + " Distance (m)", GetDistance().value());
    frc::SmartDashboard::PutNumber(name + " Angle (degrees)", GetAngle().Degrees().value());
    frc::SmartDashboard::PutNumber(name + " Velocity", GetDriveVelocity().value());
    
    frc::SmartDashboard::PutNumber(name + " Drive Voltage", GetDriveVoltage().value());
    frc::SmartDashboard::PutNumber(name +  " Turn Voltage",  GetTurnVoltage().value());
    frc::SmartDashboard::PutNumber(name + " Drive Torque Current", GetDriveTorqueCurrent().value());
    frc::SmartDashboard::PutNumber(name +  " Turn Torque Current",  GetTurnTorqueCurrent().value());
    frc::SmartDashboard::PutNumber(name + " Drive Stator Current", GetDriveStatorCurrent().value());
    frc::SmartDashboard::PutNumber(name +  " Turn Stator Current",  GetTurnStatorCurrent().value());
    frc::SmartDashboard::PutNumber(name + " Drive Supply Current", GetDriveSupplyCurrent().value());
    frc::SmartDashboard::PutNumber(name +  " Turn Supply Current",  GetTurnSupplyCurrent().value());
    frc::SmartDashboard::PutNumber(name + " Drive Motor Temp", GetDriveTemp().value());
    frc::SmartDashboard::PutNumber(name +  " Turn Motor Temp",  GetTurnTemp().value());
    frc::SmartDashboard::PutNumber(name + " Drive Processor Temp", GetDriveProcessorTemp().value());
    frc::SmartDashboard::PutNumber(name +  " Turn Processor Temp",  GetTurnProcessorTemp().value());
}

void SwerveModule::SimMode()
{
    double newAngle = frc::SmartDashboard::GetNumber(name + " Angle (degrees)", 0);
    if (newAngle != GetAngle().Degrees().value())
    {
        SetCanCoder(newAngle / 360);
    }
}