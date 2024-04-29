#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/DigitalInput.h>
#include "rev/CANSparkMax.h"
#include "Constants.h"
#include "Controls.h"
#include "subsystems/LED.h"

class Intake : frc2::SubsystemBase
{
public:
    Intake(LED *);
    void Periodic() override;
    void IntakeControls(bool, bool);
    void IntakeFromGround();
    void Purge() { SetIntakeMotor(-1.0); };
    void SetIntakeMotor(double power) { intake.Set(power); };
    void StopMotor() { SetIntakeMotor(0.0); };
    void UpdateTelemetry();
    double GetIntakeMotorRPM() { return intakeEncoder.GetVelocity(); };
    double GetGroundSpeed() { return groundSpeed; };

private:
    rev::CANSparkMax intake{RobotMap::INTAKE_MOTOR_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkRelativeEncoder intakeEncoder = intake.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

    LED *candle;

    double groundSpeed = 0.5;
};