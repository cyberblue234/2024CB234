#include "rev/CANSparkMax.h"
#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/DigitalInput.h>
#include "RobotExt.h"

class Intake : frc2::SubsystemBase
{
public:
    Intake();
    void Periodic() override;
    void IntakeFromGound();
    void SetIntakeMotor(double power) { intake.Set(power); };
    void UpdateTelemetry();
    frc2::CommandPtr GetIntakeCommand();

private:
    rev::CANSparkMax intake{RobotMap::INTAKE_MOTOR_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkRelativeEncoder intakeEncoder = intake.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    
    double groundSpeed;
};