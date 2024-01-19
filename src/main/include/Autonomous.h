#include <frc2/command/CommandPtr.h>

class Autonomous 
{
    public:
        Autonomous();
        
    private:
        void ConfigureAuto();
        std::unique_ptr<frc2::Command> onTheFly;
        std::unique_ptr<frc2::Command> followOnTheFly;
};