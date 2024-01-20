#ifndef _TELEOP_H
#define _TELEOP_H

#include <frc/Compressor.h>

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

#endif