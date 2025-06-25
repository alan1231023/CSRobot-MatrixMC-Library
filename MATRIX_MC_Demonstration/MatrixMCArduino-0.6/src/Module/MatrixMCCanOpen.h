#ifndef MATRIXMCCANOPEN_H
#define MATRIXMCCANOPEN_H

#include "CANopenNode/CANopen.h"
#include "CANopenNode_ArduinoR4/CO_app_STM32.h"
#include "CANopenNode_ArduinoR4/OD.h"
#include <FspTimer.h>

#define MMC_DEBUG_ENABLE true
#define MMC_DEBUG_SERIAL Serial
#if MMC_DEBUG_ENABLE
#    define MMC_DEBUG_HEADER()     MMC_DEBUG_SERIAL.println(F("\nBMMC\n"))
#    define MMC_DEBUG_TAIL()       MMC_DEBUG_SERIAL.println(F("\nEMMC\n"))
#    define MMC_DEBUG_PRINT(...)   MMC_DEBUG_SERIAL.print(__VA_ARGS__)
#    define MMC_DEBUG_PRINTLN(...) MMC_DEBUG_SERIAL.println(__VA_ARGS__)
#    define MMC_DEBUG_PRINT_HEADER(...)            \
        do {                                       \
            MR4_DEBUG_HEADER();                    \
            MMC_DEBUG_SERIAL.println(__VA_ARGS__); \
        } while (0)
#    define MR4_DEBUG_PRINT_TAIL(...)              \
        do {                                       \
            MMC_DEBUG_SERIAL.println(__VA_ARGS__); \
            MR4_DEBUG_TAIL();                      \
        } while (0)
#else
#    define MMC_DEBUG_HEADER()
#    define MMC_DEBUG_TAIL()
#    define MMC_DEBUG_PRINT_HEADER(...)
#    define MMC_DEBUG_PRINT_TAIL(...)
#    define MMC_DEBUG_PRINT(...)
#    define MMC_DEBUG_PRINTLN(...)
#endif

class MatrixMCCanOpen
{
public:
    enum class DevCanID
    {
        Dev1 = 0x11,
        Dev2 = 0x12,
        Dev3 = 0x13,
        Dev4 = 0x14,
    };

    enum class ODADDR
    {
        ENC_CNT            = 0x6001,   // i32
        ENC_RST            = 0x6002,   // b1
        SERVO_ANGLE        = 0x7001,   // u8
        MOTOR_SPEED        = 0x8001,   // i8
        MOTOR_DEGREE_STATE = 0x8002,   // u8
        MOTOR_DEGREE       = 0x8005,   // u16
        MOTOR_MODE         = 0x8006,   // u8
        MOTOR_BRAKE        = 0x8007,   // b1
        MOTOR_REVERSE      = 0x8008,   // b1
        SERVO_REVERSE      = 0x8009,   // b1
        PID1               = 0x9001,   // u32
        PID2               = 0x9002,   // u32
        ENC_PPR            = 0x9003,   // u16
        HBT                = 0x9004,   // u8
        HBT_INTERVAL       = 0x9005,   // u16
    };

    MatrixMCCanOpen();
    void begin(uint8_t canID, uint16_t baudrateKBPS);

    void LinkPtr_EncoderCnt(uint8_t devIdx, uint8_t motorIdx, int32_t** ENCX);
    void LinkPtr_MotorSpeed(uint8_t devIdx, uint8_t motorIdx, int8_t** MX);
    void LinkPtr_ServoAngle(uint8_t devIdx, uint8_t servoIdx, uint8_t** RCX);
    void LinkPtr_HeartbeatValue(uint8_t devIdx, uint8_t** Value);

    void SendTPDO(uint8_t devIdx);
    void Loop(void);

    CO_SDO_abortCode_t Read_SDO(
        uint8_t devIdx, uint16_t index, uint8_t subIndex, uint8_t* buf, size_t bufSize,
        size_t* readSize);
    CO_SDO_abortCode_t Write_SDO(
        uint8_t devIdx, uint16_t index, uint8_t subIndex, uint8_t* data, size_t dataSize);

private:
    CANopenNodeRenesas canOpenNodeRenesas;
    FspTimer           canTimer;

    bool beginTimer(
        FspTimer* fspTimer, float freq,
        void (*timer_callback)(timer_callback_args_t __attribute((unused)) * p_args));

    static void CanOpenInterruptCallback(timer_callback_args_t __attribute((unused)) * p_args)
    {
        canopen_app_interrupt();
    }
    static void HWInitFunction() {}
};

#endif   // MATRIXMCCANOPEN_H
