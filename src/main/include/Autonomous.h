#include <frc2/command/CommandPtr.h>

class Autonomous 
{
public:
    Autonomous();
    void AutoInit();
    void AutoControl();
    frc2::CommandPtr GetAutonomousCommand();
    void LogAutoData();
private:
    FILE *a_output;
    frc::Timer autoTimer;
    frc::Timer autoLogTimer;

};