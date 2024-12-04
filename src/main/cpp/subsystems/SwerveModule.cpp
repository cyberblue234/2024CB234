#include "subsystems/SwerveModule.h"

SwerveModule::SwerveModule(std::string name, int driveMotorID, int turnMotorID, int canCoderID, double canCoderMagnetOffset)
    : driveMotor(driveMotorID, "rio"),
      turnMotor(turnMotorID, "rio"),
      canCoder(canCoderID, "rio")
{
    this->name = name;

    SetEncoder(0);

    driveMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration driveMotorConfig{};

    driveMotorConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

    driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveMotorConfig.CurrentLimits.StatorCurrentLimit = 120.0;

    driveMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.15;
    driveMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.15;
    driveMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.15;

    // driveMotorConfig.Feedback.SensorToMechanismRatio = kDriveGearRatio;

    driveMotorConfig.Slot0.kP = kDriveP;
    driveMotorConfig.Slot0.kI = kDriveI;
    driveMotorConfig.Slot0.kD = kDriveD;
    driveMotorConfig.Slot0.kS = kDrive_kS.value();
    driveMotorConfig.Slot0.kV = kDrive_kV.value();

    driveMotor.GetConfigurator().Apply(driveMotorConfig);

    turnMotor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration turnMotorConfig{};

    turnMotorConfig.Feedback.FeedbackRemoteSensorID = canCoder.GetDeviceID();
    turnMotorConfig.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::FusedCANcoder;
    turnMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
    // turnMotorConfig.Feedback.RotorToSensorRatio = kTurnGearRatio;

    turnMotorConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;

    turnMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnMotorConfig.CurrentLimits.StatorCurrentLimit = 120.0;

    turnMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.15;
    turnMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.15;
    turnMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.15;

    turnMotorConfig.Slot0.kP = kTurnP;
    turnMotorConfig.Slot0.kI = kTurnI;
    turnMotorConfig.Slot0.kD = kTurnD;

    // TODO: find replacement -- try configs::MotorOutputConfigs::WithPeakForwardDutyCycle and configs::MotorOutputConfigs::WithPeakReverseDutyCycle
    // turnMotor.ConfigVoltageCompSaturation(11.0);
    // turnMotor.EnableVoltageCompensation(true);

    turnMotor.GetConfigurator().Apply(turnMotorConfig);

    canCoder.GetConfigurator().Apply(configs::CANcoderConfiguration{});
    configs::CANcoderConfiguration canCoderConfig{};

    canCoderConfig.MagnetSensor.MagnetOffset = canCoderMagnetOffset;
    canCoderConfig.MagnetSensor.AbsoluteSensorRange = signals::AbsoluteSensorRangeValue::Unsigned_0To1;

    canCoder.GetConfigurator().Apply(canCoderConfig);
    

    wpi::SendableRegistry::SetName(&driveMotor, name, "Drive");
    wpi::SendableRegistry::SetName(&turnMotor, name, "Turn");
    wpi::SendableRegistry::SetName(&canCoder, name, "CANCoder");
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
    controls::PositionVoltage& turnPos = turnPositionOut.WithPosition(units::angle::turn_t(state.angle.Degrees().value() / 360));
    turnMotor.SetControl(turnPos.WithSlot(0));
    // Set the motor outputs.
    auto setVelocity = units::angular_velocity::turns_per_second_t(state.speed.value() / kDriveDistanceRatio);
    controls::VelocityVoltage& driveVelocity = driveVelocityOut.WithVelocity(setVelocity);
    driveMotor.SetControl(driveVelocity.WithSlot(0));

    TelemetryHelperNumber("SetSpeed", state.speed.value());
    TelemetryHelperNumber("SetMotorSpeed", setVelocity.value());
    TelemetryHelperNumber("SetAngle", state.angle.Degrees().value());
    TelemetryHelperNumber("Drive FF", driveMotor.GetClosedLoopFeedForward().GetValue());
}

void SwerveModule::UpdateTelemetry()
{
    TelemetryHelperNumber("Distance (m)", GetDistance().value());
    TelemetryHelperNumber("Angle (degrees)", GetAngle().Degrees().value());
    TelemetryHelperNumber("Raw Cancoder", GetRawCANcoderPosition().value());
    TelemetryHelperNumber("Velocity", GetVelocity().value());
    TelemetryHelperNumber("Drive Output Voltage", GetDriveOutputVoltage().value());
    TelemetryHelperNumber("Turn Output Voltage",  GetTurnOutputVoltage().value());
    TelemetryHelperNumber("Drive Torque Current", GetDriveTorqueCurrent().value());
    TelemetryHelperNumber("Turn Torque Current",  GetTurnTorqueCurrent().value());
    TelemetryHelperNumber("Drive Stator Current", GetDriveStatorCurrent().value());
    TelemetryHelperNumber("Turn Stator Current",  GetTurnStatorCurrent().value());
    TelemetryHelperNumber("Drive Supply Current", GetDriveSupplyCurrent().value());
    TelemetryHelperNumber("Turn Supply Current",  GetTurnSupplyCurrent().value());
    TelemetryHelperNumber("Drive Motor Temp", GetDriveTemp().value());
    TelemetryHelperNumber("Turn Motor Temp",  GetTurnTemp().value());
    TelemetryHelperNumber("Drive Processor Temp", GetDriveProcessorTemp().value());
    TelemetryHelperNumber("Turn Processor Temp",  GetTurnProcessorTemp().value());
}

void SwerveModule::SimMode()
{
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
    turnMotorSim.SetRawRotorPosition(kTurnGearRatio * turnMotorSimModel.GetAngularPosition());
    double intp;
    auto canCoderPos = std::modf(turnMotorSimModel.GetAngularPosition().value(), &intp);
    if (canCoderPos < -0.5) canCoderPos += 1;
    else if (canCoderPos > 0.5) canCoderPos -= 1;
    canCoderSim.SetRawPosition(units::angle::turn_t(canCoderPos));
    turnMotorSim.SetRotorVelocity(kTurnGearRatio * turnMotorSimModel.GetAngularVelocity());
    canCoderSim.SetVelocity(turnMotorSimModel.GetAngularVelocity());

    
}