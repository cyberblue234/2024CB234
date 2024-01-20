#include <frc2/command/CommandPtr.h>

class Autonomous 
{
public:
    void ConfigureAuto(); 
    std::unique_ptr<frc2::Command> onTheFly;
private:
    
    
    std::unique_ptr<frc2::Command> followOnTheFly;
};