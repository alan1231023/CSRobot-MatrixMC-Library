#include "MatrixMCDC.h"

MatrixMCDC::MatrixMCDC() {}

void MatrixMCDC::begin(uint8_t devIdx, uint8_t motorIdx, MatrixMCCanOpen* mcCanOpen)
{
    this->_devIdx    = devIdx;
    this->_motorIdx  = motorIdx;
    this->_mcCanOpen = mcCanOpen;

    this->_mcCanOpen->LinkPtr_EncoderCnt(devIdx, motorIdx, &this->counterPtr);
    this->_mcCanOpen->LinkPtr_MotorSpeed(devIdx, motorIdx, &this->speedPtr);

    this->_prevMode   = Mode::NORMAL;
    this->_prevDegree = 0;
}

bool MatrixMCDC::setReverse(bool reverse)
{
    CO_SDO_abortCode_t ret = _mcCanOpen->Write_SDO(
        _devIdx,
        (uint16_t)MatrixMCCanOpen::ODADDR::MOTOR_REVERSE,
        _motorIdx,
        (uint8_t*)&reverse,
        1);
    return ret == CO_SDO_AB_NONE;
}

bool MatrixMCDC::setPower(int16_t power)
{
    if (_prevMode != Mode::NORMAL) {
        if (ChangeMode(Mode::NORMAL) == false) {
            return false;
        }
    }
    if (power > 100) power = 100;
    if (power < -100) power = -100;
    if (*speedPtr != (int8_t)power) {
        // _mcCanOpen->SendTPDO(_devIdx);
        CO_SDO_abortCode_t ret = _mcCanOpen->Write_SDO(
            _devIdx,
            (uint16_t)MatrixMCCanOpen::ODADDR::MOTOR_SPEED,
            _motorIdx,
            (uint8_t*)&power,
            1);

        if (ret == CO_SDO_AB_NONE) {
            *speedPtr = (int8_t)power;
            return true;
        }
    }
    return false;
}

bool MatrixMCDC::setSpeed(int16_t speed)
{
    if (_prevMode != Mode::FIX_SPEED) {
        if (ChangeMode(Mode::FIX_SPEED) == false) {
            return false;
        }
    }
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
    if (*speedPtr != (int8_t)speed) {
        // _mcCanOpen->SendTPDO(_devIdx);
        CO_SDO_abortCode_t ret = _mcCanOpen->Write_SDO(
            _devIdx,
            (uint16_t)MatrixMCCanOpen::ODADDR::MOTOR_SPEED,
            _motorIdx,
            (uint8_t*)&speed,
            1);

        if (ret == CO_SDO_AB_NONE) {
            *speedPtr = (int8_t)speed;
            return true;
        }
    }
    return false;
}

bool MatrixMCDC::rotateFor(int16_t speed, uint16_t degree)
{
    if (_prevMode != Mode::ROTATE) {
        if (ChangeMode(Mode::ROTATE) == false) {
            return false;
        }
    }

    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
    *speedPtr = (int8_t)speed;
    _mcCanOpen->SendTPDO(_devIdx);

    CO_SDO_abortCode_t ret = _mcCanOpen->Write_SDO(
        _devIdx, (uint16_t)MatrixMCCanOpen::ODADDR::MOTOR_DEGREE, _motorIdx, (uint8_t*)&degree, 2);

    uint8_t            cmd  = 1;
    CO_SDO_abortCode_t ret1 = _mcCanOpen->Write_SDO(
        _devIdx,
        (uint16_t)MatrixMCCanOpen::ODADDR::MOTOR_DEGREE_STATE,
        _motorIdx,
        (uint8_t*)&cmd,
        1);

    if (ret == CO_SDO_AB_NONE && ret1 == CO_SDO_AB_NONE) {
        _prevDegree = degree;
        return true;
    }
    return false;
}

int32_t MatrixMCDC::getCounter(void)
{
    int32_t            counter  = 0;
    size_t             readSize = 0;
    CO_SDO_abortCode_t ret      = _mcCanOpen->Read_SDO(
        _devIdx,
        (uint16_t)MatrixMCCanOpen::ODADDR::ENC_CNT,
        _motorIdx,
        (uint8_t*)&counter,
        4,
        (size_t*)&readSize);
    if (ret == CO_SDO_AB_NONE && readSize == 4) {
        return counter;
    }
    return 0;
}

int32_t MatrixMCDC::getDegrees(void)
{
    return (int32_t)((double)getCounter() / _EncPPR * 360);
}

bool MatrixMCDC::resetCounter(void)
{
    bool               tmp = true;
    CO_SDO_abortCode_t ret = _mcCanOpen->Write_SDO(
        _devIdx, (uint16_t)MatrixMCCanOpen::ODADDR::ENC_RST, _motorIdx, (uint8_t*)&tmp, 1);

    return ret == CO_SDO_AB_NONE;
}

bool MatrixMCDC::setBrake(bool brake)
{
    if (brake) {
        CO_SDO_abortCode_t ret = _mcCanOpen->Write_SDO(
            _devIdx,
            (uint16_t)MatrixMCCanOpen::ODADDR::MOTOR_BRAKE,
            _motorIdx,
            (uint8_t*)&brake,
            1);
        return ret == CO_SDO_AB_NONE && setPower(0);
    } else {
        return setPower(0);
    }
}

bool MatrixMCDC::setFixSpeedPID(float kp, float ki, float kd)
{
    PIDConfig_u pidConfig;
    pidConfig.all = 0;
    pidConfig.kp  = (uint16_t)(kp * 100);
    pidConfig.ki  = (uint16_t)(ki * 100);
    pidConfig.kd  = (uint16_t)(kd * 100);

    CO_SDO_abortCode_t ret = _mcCanOpen->Write_SDO(
        _devIdx, (uint16_t)MatrixMCCanOpen::ODADDR::PID1, _motorIdx, (uint8_t*)&pidConfig.all, 4);

    return ret == CO_SDO_AB_NONE;
}

bool MatrixMCDC::setRotatePID(float kp, float ki, float kd)
{
    PIDConfig_u pidConfig;
    pidConfig.all = 0;
    pidConfig.kp  = (uint16_t)(kp * 100);
    pidConfig.ki  = (uint16_t)(ki * 100);
    pidConfig.kd  = (uint16_t)(kd * 100);

    CO_SDO_abortCode_t ret = _mcCanOpen->Write_SDO(
        _devIdx, (uint16_t)MatrixMCCanOpen::ODADDR::PID2, _motorIdx, (uint8_t*)&pidConfig.all, 4);
    return ret == CO_SDO_AB_NONE;
}

bool MatrixMCDC::ChangeMode(Mode mode)
{
    CO_SDO_abortCode_t ret = _mcCanOpen->Write_SDO(
        _devIdx, (uint16_t)MatrixMCCanOpen::ODADDR::MOTOR_MODE, _motorIdx, (uint8_t*)&mode, 1);

    if (ret == CO_SDO_AB_NONE) {
        _prevMode = mode;
        return true;
    }
    return false;
}

bool MatrixMCDC::setEncPPR(uint16_t ppr)
{
    CO_SDO_abortCode_t ret = _mcCanOpen->Write_SDO(
        _devIdx, (uint16_t)MatrixMCCanOpen::ODADDR::ENC_PPR, _motorIdx, (uint8_t*)&ppr, 2);
    if (ret == CO_SDO_AB_NONE) {
        _EncPPR = ppr;
        return true;
    }
    return false;
}
