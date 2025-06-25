#ifndef MATRIXMCDEV_H
#define MATRIXMCDEV_H

#include "MatrixMCCanOpen.h"
#include "MatrixMCDC.h"
#include "MatrixMCRC.h"

class MatrixMCDev
{
public:
    enum class MotorNum
    {
        M1 = 1,
        M2 = 2,
        M3 = 3,
        M4 = 4,
    };

    enum class ServoNum
    {
        RC1 = 1,
        RC2 = 2,
    };

    MatrixMCDev();
    void begin(uint8_t devIdx, MatrixMCCanOpen* mcCanOpen);
    bool SetHeartbeatInterval(uint32_t intervalMs);
    bool CheckHeartbeat(void);

    MatrixMCDC M1;
    MatrixMCDC M2;
    MatrixMCDC M3;
    MatrixMCDC M4;

    MatrixMCRC RC1;
    MatrixMCRC RC2;

private:
    uint8_t          _devIdx;
    MatrixMCCanOpen* _mcCanOpen;

    uint32_t _hbIntervalMs;
    uint32_t _hbTimer;
    uint8_t* _hbValuePtr;
};

#endif   // MATRIXMCDEV_H
