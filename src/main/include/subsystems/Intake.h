#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/DigitalInput.h>
#include "rev/CANSparkMax.h"
#include "Constants.h"

class Intake : frc2::SubsystemBase
{
public:
    Intake();
    void Periodic() override;
    void IntakeFromGround();
    void Purge() { SetIntakeMotor(-1.0); };
    void SetIntakeMotor(double power) { intake.Set(power); };
    void UpdateTelemetry();
    double GetGroundSpeed() { return groundSpeed; };

private:
    rev::CANSparkMax intake{RobotMap::INTAKE_MOTOR_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkRelativeEncoder intakeEncoder = intake.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

    double groundSpeed = 0.5;
};