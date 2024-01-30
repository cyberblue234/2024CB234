#include "rev/CANSparkMax.h"
#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <frc/DutyCycleEncoder.h>


class Shooter
{
public:
    Shooter();
    void ShooterControl();
    void SetShooterMotor1(double power) { shooter1.Set(power); };
    void SetShooterMotor2(double power) { shooter2.Set(power); };

private:
    rev::CANSparkMax shooter1{RobotMap::SHOOTER_MOTOR1_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax shooter2{RobotMap::SHOOTER_MOTOR2_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkRelativeEncoder shooter1Encoder = shooter1.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::SparkRelativeEncoder shooter2Encoder = shooter2.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::SparkMaxPIDController shooter1PID = shooter1.GetPIDController();
    rev::SparkMaxPIDController shooter2PID = shooter2.GetPIDController();

    frc::DutyCycleEncoder absEncoder{1};

    double power1;
    double power2;
};