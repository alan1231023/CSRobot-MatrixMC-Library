#ifndef MatrixMCRC_H
#define MatrixMCRC_H

#include "MatrixMCCanOpen.h"

class MatrixMCRC
{
public:
    MatrixMCRC();
    void begin(uint8_t devIdx, uint8_t servoIdx, MatrixMCCanOpen* mcCanOpen);
    bool setReverse(bool reverse);
    bool setAngle(uint8_t angle);

private:
    MatrixMCCanOpen* _mcCanOpen;

    uint8_t  _devIdx;
    uint8_t  _servoIdx;
    uint8_t* anglePtr;
};

#endif   // MatrixMCRC_H
