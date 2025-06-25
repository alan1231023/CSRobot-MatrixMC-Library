#include "MatrixMCRC.h"

MatrixMCRC::MatrixMCRC() {}

void MatrixMCRC::begin(uint8_t devIdx, uint8_t servoIdx, MatrixMCCanOpen* mcCanOpen)
{
    _devIdx    = devIdx;
    _servoIdx  = servoIdx;
    _mcCanOpen = mcCanOpen;

    _mcCanOpen->LinkPtr_ServoAngle(devIdx, servoIdx, &anglePtr);
}

bool MatrixMCRC::setReverse(bool reverse)
{
    CO_SDO_abortCode_t ret = _mcCanOpen->Write_SDO(
        _devIdx,
        (uint16_t)MatrixMCCanOpen::ODADDR::SERVO_REVERSE,
        _servoIdx,
        (uint8_t*)&reverse,
        1);

    return ret == CO_SDO_AB_NONE;
}

bool MatrixMCRC::setAngle(uint8_t angle)
{
    if (*anglePtr != angle) {
        *anglePtr = angle;
        // _mcCanOpen->SendTPDO(_devIdx);
        CO_SDO_abortCode_t ret = _mcCanOpen->Write_SDO(
            _devIdx,
            (uint16_t)MatrixMCCanOpen::ODADDR::SERVO_ANGLE,
            _servoIdx,
            (uint8_t*)&angle,
            1);

        return ret == CO_SDO_AB_NONE;
    }
    return false;
}
