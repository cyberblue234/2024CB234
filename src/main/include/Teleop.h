#include <frc/Timer.h>

class Teleop
{
public:
    Teleop();
    void TeleopInit();
    void OperatorControls();
    void LogTeleopData();

private:
    FILE *t_output;
    frc::Timer logTimer;
};
