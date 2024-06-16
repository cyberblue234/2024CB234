#include "subsystems/Elevator.h"

bool elevator1Registered = false;
bool elevator2Registered = false;

Elevator::Elevator()
{
    elevator1Motor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration elevator1Config{};

    configs::MotorOutputConfigs elevator1MotorOutput{};
    elevator1MotorOutput.WithInverted(signals::InvertedValue::Clockwise_Positive);
    elevator1MotorOutput.WithNeutralMode(signals::NeutralModeValue::Brake);
    elevator1Config.WithMotorOutput(elevator1MotorOutput);

    elevator1Motor.GetConfigurator().Apply(elevator1Config);

    elevator2Motor.GetConfigurator().Apply(configs::TalonFXConfiguration{});
    configs::TalonFXConfiguration elevator2Config{};

    configs::MotorOutputConfigs elevator2MotorOutput{};
    elevator2MotorOutput.WithNeutralMode(signals::NeutralModeValue::Brake);
    elevator2Config.WithMotorOutput(elevator2MotorOutput);

    elevator2Motor.GetConfigurator().Apply(elevator2Config);
}

void Elevator::Periodic()
{
    if (GetElevator1BottomLimit() && (GetElevator1Encoder() > 0.025 || GetElevator1Encoder() < -0.025))
    {
        ResetElevator1Encoder();
        elevator1Registered = true;
    }
    if (GetElevator2BottomLimit() && (GetElevator2Encoder() > 0.025 || GetElevator2Encoder() < -0.025))
    {
        ResetElevator2Encoder();
        elevator2Registered = true;
    }
    UpdateTelemetry();
}

void Elevator::ElevatorControl(double value)
{
    bool directionTest;
    directionTest = value > 0;

    bool elevator1Limit = directionTest ? GetElevator1Encoder() > GetHardEncoderLimit() : GetElevator1BottomLimit() == true || (GetElevator1Encoder() < -2.0 && elevator1Registered == true);
    bool elevator2Limit = directionTest ? GetElevator2Encoder() > GetHardEncoderLimit() : GetElevator2BottomLimit() == true || (GetElevator2Encoder() < -2.0 && elevator2Registered == true);

    if (elevator1Limit == false)
        SetElevator1Motor(value);
    else
        SetElevator1Motor(0.0);
    if (elevator2Limit == false)
        SetElevator2Motor(value);
    else
        SetElevator2Motor(0.0);
}

void Elevator::UpdateTelemetry()
{ 
    frc::SmartDashboard::PutNumber("Elevator 1 Encoder Pos", GetElevator1Encoder());
    frc::SmartDashboard::PutNumber("Elevator 2 Encoder Pos", GetElevator2Encoder());
    frc::SmartDashboard::PutBoolean("Limit Switch 1", GetElevator1BottomLimit());
    frc::SmartDashboard::PutBoolean("Limit Switch 2", GetElevator2BottomLimit());
}