#include "MatrixMCDev.h"

MatrixMCDev::MatrixMCDev() {}

void MatrixMCDev::begin(uint8_t devIdx, MatrixMCCanOpen* mcCanOpen)
{
    _devIdx       = devIdx;
    _mcCanOpen    = mcCanOpen;
    _hbIntervalMs = 2000;

    M1.begin(devIdx, (uint8_t)MotorNum::M1, mcCanOpen);
    M2.begin(devIdx, (uint8_t)MotorNum::M2, mcCanOpen);
    M3.begin(devIdx, (uint8_t)MotorNum::M3, mcCanOpen);
    M4.begin(devIdx, (uint8_t)MotorNum::M4, mcCanOpen);

    RC1.begin(devIdx, (uint8_t)ServoNum::RC1, mcCanOpen);
    RC2.begin(devIdx, (uint8_t)ServoNum::RC2, mcCanOpen);

    this->_mcCanOpen->LinkPtr_HeartbeatValue(devIdx, &this->_hbValuePtr);
}

bool MatrixMCDev::CheckHeartbeat(void)
{
    bool               hbt = true;
    CO_SDO_abortCode_t ret = _mcCanOpen->Write_SDO(
        _devIdx, (uint16_t)MatrixMCCanOpen::ODADDR::HBT, 0, (uint8_t*)&hbt, 1);
    return ret == CO_SDO_AB_NONE;
}

bool MatrixMCDev::SetHeartbeatInterval(uint32_t intervalMs)
{
    CO_SDO_abortCode_t ret = _mcCanOpen->Write_SDO(
        _devIdx, (uint16_t)MatrixMCCanOpen::ODADDR::HBT_INTERVAL, 0, (uint8_t*)&intervalMs, 2);
    if (ret == CO_SDO_AB_NONE) {
        _hbIntervalMs = intervalMs * 2 / 3;
        return true;
    }
    return false;
}
