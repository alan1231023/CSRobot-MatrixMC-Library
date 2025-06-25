#include "MatrixMC.h"

MatrixMC::MatrixMC(bool enableMC1, bool enableMC2, bool enableMC3, bool enableMC4)
    : _enableMC1(enableMC1)
    , _enableMC2(enableMC2)
    , _enableMC3(enableMC3)
    , _enableMC4(enableMC4)
{}

bool MatrixMC::begin(void)
{
    this->MCCanOpen.begin(0x10, 500);
    this->MC1.begin((uint8_t)DevNum::_1, &this->MCCanOpen);
    this->MC2.begin((uint8_t)DevNum::_2, &this->MCCanOpen);
    this->MC3.begin((uint8_t)DevNum::_3, &this->MCCanOpen);
    this->MC4.begin((uint8_t)DevNum::_4, &this->MCCanOpen);

    uint32_t timeout = millis() + 1000;
    while (millis() < timeout) {
        this->loop();
    }
}

bool MatrixMC::loop()
{
    MCCanOpen.Loop();

    static uint32_t timer = 0;
    if (millis() > timer) {
        timer = millis() + 500;
        if (_enableMC1) {
            MC1.CheckHeartbeat();
        }
        if (_enableMC2) {
            MC2.CheckHeartbeat();
        }
        if (_enableMC3) {
            MC3.CheckHeartbeat();
        }
        if (_enableMC4) {
            MC4.CheckHeartbeat();
        }
    }
    return true;
}
