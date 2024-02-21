#include "rev/CANSparkMax.h"
#include "Constants.h"
#include <frc/DutyCycleEncoder.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/Timer.h>
#include <frc/DigitalInput.h>
#include "RobotExt.h"

class Shooter : frc2::SubsystemBase
{
public:
    Shooter();
    void ShootAtSpeaker();
    void ShootAtAmp();
    void IntakeFromSource();

    void UpdateTelemetry();

    void SetShooterMotors(double power) { SetShooterMotor1(power); SetShooterMotor2(power); };
    void SetShooterMotor1(double power) { shooter1Motor.Set(power); };
    void SetShooterMotor2(double power) { shooter2Motor.Set(power); };

    double GetShooterAngle() { return shooterAngleEncoder.GetDistance(); };
    double GetAverageRPM() { return (GetShooter1RPM() + GetShooter2RPM()) / 2.0; };
    double GetShooter1RPM() { return shooter1Encoder.GetVelocity(); };
    double GetShooter2RPM() { return shooter2Encoder.GetVelocity(); };
    double GetSpeakerSpeed() { return speakerSpeed; };
    double GetAmpSpeed() { return ampSpeed; };
    double GetIntakeSpeed() { return intakeSpeed; };

    frc2::CommandPtr GetShooterCommand();

    // FOR DEBUGGING
    bool shootAtSpeaker = true;

private:
    rev::CANSparkMax shooter1Motor{RobotMap::SHOOTER_MOTOR1_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax shooter2Motor{RobotMap::SHOOTER_MOTOR2_ADDRESS, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkRelativeEncoder shooter1Encoder = shooter1Motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::SparkRelativeEncoder shooter2Encoder = shooter2Motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::SparkPIDController shooter1PID = shooter1Motor.GetPIDController();
    rev::SparkPIDController shooter2PID = shooter2Motor.GetPIDController();

    frc::DutyCycleEncoder shooterAngleEncoder{RobotMap::SHOOTER_ENCODER_ADDRESS};

    double speakerSpeed;
    double ampSpeed;
    double intakeSpeed;
};