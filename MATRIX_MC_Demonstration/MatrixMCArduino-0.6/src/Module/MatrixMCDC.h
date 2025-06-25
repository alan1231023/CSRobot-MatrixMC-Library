#ifndef MatrixMCDC_H
#define MatrixMCDC_H

#include "MatrixMCCanOpen.h"

class MatrixMCDC
{
public:
    typedef union
    {
        uint32_t all;
        struct
        {
            uint32_t kp : 10;
            uint32_t ki : 10;
            uint32_t kd : 10;
            uint32_t : 2;
        };
    } PIDConfig_u;

    enum class Mode
    {
        NORMAL,
        FIX_SPEED,
        ROTATE,
    };

    MatrixMCDC();

    void    begin(uint8_t devIdx, uint8_t motorIdx, MatrixMCCanOpen* mcCanOpen);
    bool    setReverse(bool dir);
    bool    setPower(int16_t power);
    bool    setSpeed(int16_t speed);
    bool    rotateFor(int16_t speed, uint16_t degree);
    int32_t getCounter(void);
    int32_t getDegrees(void);
    bool    resetCounter(void);
    bool    setBrake(bool brake);
    bool    setFixSpeedPID(float kp, float ki, float kd);
    bool    setRotatePID(float kp, float ki, float kd);
    bool    setEncPPR(uint16_t ppr);


private:
    bool             ChangeMode(Mode mode);
    MatrixMCCanOpen* _mcCanOpen;

    uint8_t  _devIdx;
    uint8_t  _motorIdx;
    int32_t* counterPtr;
    int8_t*  speedPtr;
    Mode     _prevMode;
    uint16_t _prevDegree;
    uint16_t _EncPPR;
};

#endif   // MatrixMCDC_H
