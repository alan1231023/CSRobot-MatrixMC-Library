#ifndef MATRIXMC_H
#define MATRIXMC_H

#include "Module/MatrixMCCanOpen.h"
#include "Module/MatrixMCDev.h"

class MatrixMC
{
public:
    enum class DevNum
    {
        _1 = 1,
        _2 = 2,
        _3 = 3,
        _4 = 4,
    };

    MatrixMC(
        bool enableMC1 = false, bool enableMC2 = false, bool enableMC3 = false,
        bool enableMC4 = false);
    bool begin();
    bool loop();

    MatrixMCDev MC1;
    MatrixMCDev MC2;
    MatrixMCDev MC3;
    MatrixMCDev MC4;

private:
    MatrixMCCanOpen MCCanOpen;
    bool            _enableMC1;
    bool            _enableMC2;
    bool            _enableMC3;
    bool            _enableMC4;
};

#endif   // MATRIXMC_H
