#include "MatrixMCCanOpen.h"

MatrixMCCanOpen::MatrixMCCanOpen() {}

void MatrixMCCanOpen::begin(uint8_t canID, uint16_t baudrateKBPS)
{
#if MMC_DEBUG_ENABLE
    MMC_DEBUG_SERIAL.begin(115200);
#endif

    // RPDO
    /* Device 00 ID: 0x11 */
    OD_PERSIST_COMM.x1400_RPDOCommunicationParameter.COB_IDUsedByRPDO =
        CO_CAN_ID_TPDO_1 + (uint8_t)MatrixMCCanOpen::DevCanID::Dev1;
    OD_PERSIST_COMM.x1401_RPDOCommunicationParameter.COB_IDUsedByRPDO =
        CO_CAN_ID_TPDO_2 + (uint8_t)MatrixMCCanOpen::DevCanID::Dev1;
    // /* Device 01 ID: 0x12 */
    OD_PERSIST_COMM.x1402_RPDOCommunicationParameter.COB_IDUsedByRPDO =
        CO_CAN_ID_TPDO_1 + (uint8_t)MatrixMCCanOpen::DevCanID::Dev2;
    OD_PERSIST_COMM.x1403_RPDOCommunicationParameter.COB_IDUsedByRPDO =
        CO_CAN_ID_TPDO_2 + (uint8_t)MatrixMCCanOpen::DevCanID::Dev2;
    // /* Device 02 ID: 0x13 */
    OD_PERSIST_COMM.x1404_RPDOCommunicationParameter.COB_IDUsedByRPDO =
        CO_CAN_ID_TPDO_1 + (uint8_t)MatrixMCCanOpen::DevCanID::Dev3;
    OD_PERSIST_COMM.x1405_RPDOCommunicationParameter.COB_IDUsedByRPDO =
        CO_CAN_ID_TPDO_2 + (uint8_t)MatrixMCCanOpen::DevCanID::Dev3;
    // /* Device 03 ID: 0x14 */
    OD_PERSIST_COMM.x1406_RPDOCommunicationParameter.COB_IDUsedByRPDO =
        CO_CAN_ID_TPDO_1 + (uint8_t)MatrixMCCanOpen::DevCanID::Dev4;
    OD_PERSIST_COMM.x1407_RPDOCommunicationParameter.COB_IDUsedByRPDO =
        CO_CAN_ID_TPDO_2 + (uint8_t)MatrixMCCanOpen::DevCanID::Dev4;

    // TPDO
    /* Device 00 ID: 0x11 */
    OD_PERSIST_COMM.x1800_TPDOCommunicationParameter.COB_IDUsedByTPDO =
        CO_CAN_ID_RPDO_1 + (uint8_t)MatrixMCCanOpen::DevCanID::Dev1;
    /* Device 01 ID: 0x12 */
    OD_PERSIST_COMM.x1801_TPDOCommunicationParameter.COB_IDUsedByTPDO =
        CO_CAN_ID_RPDO_1 + (uint8_t)MatrixMCCanOpen::DevCanID::Dev2;
    /* Device 02 ID: 0x13 */
    OD_PERSIST_COMM.x1802_TPDOCommunicationParameter.COB_IDUsedByTPDO =
        CO_CAN_ID_RPDO_1 + (uint8_t)MatrixMCCanOpen::DevCanID::Dev3;
    /* Device 03 ID: 0x14 */
    OD_PERSIST_COMM.x1803_TPDOCommunicationParameter.COB_IDUsedByTPDO =
        CO_CAN_ID_RPDO_1 + (uint8_t)MatrixMCCanOpen::DevCanID::Dev4;

    canOpenNodeRenesas.CANHandle      = &MatrixMC_CAN;
    canOpenNodeRenesas.HWInitFunction = HWInitFunction;
    canOpenNodeRenesas.desiredNodeID  = canID;
    canOpenNodeRenesas.baudrate       = baudrateKBPS;
    bool res                          = canopen_app_init(&canOpenNodeRenesas) == 0;
    // res &= beginTimer(&canTimer, 1000, CanOpenInterruptCallback);

    if (res) {
        MMC_DEBUG_PRINTLN("MatrixMC Init Success");
    } else {
        MMC_DEBUG_PRINTLN("MatrixMC Init Failed");
    }
}

void MatrixMCCanOpen::LinkPtr_EncoderCnt(uint8_t devIdx, uint8_t motorIdx, int32_t** ENCX)
{
    switch (devIdx) {
    case 1: *ENCX = &OD_PERSIST_COMM.x6001_dev1_ENCArr[motorIdx - 1]; break;
    case 2: *ENCX = &OD_PERSIST_COMM.x6002_dev2_ENCArr[motorIdx - 1]; break;
    case 3: *ENCX = &OD_PERSIST_COMM.x6003_dev3_ENCArr[motorIdx - 1]; break;
    case 4: *ENCX = &OD_PERSIST_COMM.x6004_dev4_ENCArr[motorIdx - 1]; break;
    default: break;
    }
}

void MatrixMCCanOpen::LinkPtr_MotorSpeed(uint8_t devIdx, uint8_t motorIdx, int8_t** MX)
{
    switch (devIdx) {
    case 1: *MX = &OD_PERSIST_COMM.x8001_dev1_MotorSpeedArr[motorIdx - 1]; break;
    case 2: *MX = &OD_PERSIST_COMM.x8002_dev2_MotorSpeedArr[motorIdx - 1]; break;
    case 3: *MX = &OD_PERSIST_COMM.x8003_dev3_MotorSpeedArr[motorIdx - 1]; break;
    case 4: *MX = &OD_PERSIST_COMM.x8004_dev4_MotorSpeedArr[motorIdx - 1]; break;
    default: break;
    }
}

void MatrixMCCanOpen::LinkPtr_ServoAngle(uint8_t devIdx, uint8_t servoIdx, uint8_t** RCX)
{
    switch (devIdx) {
    case 1: *RCX = &OD_PERSIST_COMM.x7001_dev1_RCArr[servoIdx - 1]; break;
    case 2: *RCX = &OD_PERSIST_COMM.x7002_dev2_RCArr[servoIdx - 1]; break;
    case 3: *RCX = &OD_PERSIST_COMM.x7003_dev3_RCArr[servoIdx - 1]; break;
    case 4: *RCX = &OD_PERSIST_COMM.x7004_dev4_RCArr[servoIdx - 1]; break;
    default: break;
    }
}

void MatrixMCCanOpen::LinkPtr_HeartbeatValue(uint8_t devIdx, uint8_t** Value)
{
    switch (devIdx) {
    case 1: *Value = &OD_PERSIST_COMM.x9001_dev1_HeartbeatValue; break;
    case 2: *Value = &OD_PERSIST_COMM.x9002_dev2_HeartbeatValue; break;
    case 3: *Value = &OD_PERSIST_COMM.x9003_dev3_HeartbeatValue; break;
    case 4: *Value = &OD_PERSIST_COMM.x9004_dev4_HeartbeatValue; break;
    default: break;
    }
}

void MatrixMCCanOpen::SendTPDO(uint8_t devIdx)
{
    static uint32_t timer = 0;
    if (micros() - timer < 2000) {
        delayMicroseconds(2000);
    }

    CO_TPDOsendRequest(&(canOpenNodeRenesas.canOpenStack->TPDO[devIdx - 1]));
    timer = micros();
}

bool MatrixMCCanOpen::beginTimer(
    FspTimer* fspTimer, float freq,
    void (*timer_callback)(timer_callback_args_t __attribute((unused)) * p_args))
{
    uint8_t timer_type = GPT_TIMER;
    int8_t  tindex     = FspTimer::get_available_timer(timer_type);
    if (tindex < 0) {
        tindex = FspTimer::get_available_timer(timer_type, true);
    }
    if (tindex < 0) {
        return false;
    }

    FspTimer::force_use_of_pwm_reserved_timer();

    if (!fspTimer->begin(TIMER_MODE_PERIODIC, timer_type, tindex, freq, 0.0f, timer_callback)) {
        return false;
    }

    if (!fspTimer->setup_overflow_irq()) {
        return false;
    }

    if (!fspTimer->open()) {
        return false;
    }

    if (!fspTimer->start()) {
        return false;
    }
    return true;
}

void MatrixMCCanOpen::Loop(void)
{
    static CO_NMT_internalState_t state = CO_NMT_UNKNOWN;

    canopen_app_process();

    if (state != canOpenNodeRenesas.canOpenStack->NMT->operatingState) {
        switch (canOpenNodeRenesas.canOpenStack->NMT->operatingState) {
        case CO_NMT_INITIALIZING: MMC_DEBUG_PRINTLN("CO_NMT_INITIALIZING\n"); break;
        case CO_NMT_PRE_OPERATIONAL: MMC_DEBUG_PRINTLN("CO_NMT_PRE_OPERATIONAL\n"); break;
        case CO_NMT_OPERATIONAL: MMC_DEBUG_PRINTLN("CO_NMT_OPERATIONAL\n"); break;
        case CO_NMT_STOPPED: MMC_DEBUG_PRINTLN("CO_NMT_STOPPED\n"); break;
        case CO_NMT_UNKNOWN: MMC_DEBUG_PRINTLN("CO_NMT_UNKNOWN\n"); break;
        default: MMC_DEBUG_PRINTLN("Unknown state\n"); break;
        }
        state = canOpenNodeRenesas.canOpenStack->NMT->operatingState;
    }
}

CO_SDO_abortCode_t MatrixMCCanOpen::Read_SDO(
    uint8_t devIdx, uint16_t index, uint8_t subIndex, uint8_t* buf, size_t bufSize,
    size_t* readSize)
{
    static uint8_t lastNodeId = 0xFF;

    uint8_t         nodeId = (uint8_t)DevCanID::Dev1 + devIdx - 0x01;
    CO_SDO_return_t SDO_ret;

    if (nodeId != lastNodeId) {
        // setup client (this can be skipped, if remote device don't change)
        SDO_ret = CO_SDOclient_setup(
            canOpenNodeRenesas.canOpenStack->SDOclient,
            CO_CAN_ID_SDO_CLI + nodeId,
            CO_CAN_ID_SDO_SRV + nodeId,
            nodeId);
        if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
            return CO_SDO_AB_GENERAL;
        }
        lastNodeId = nodeId;
    }

    // initiate upload
    SDO_ret = CO_SDOclientUploadInitiate(
        canOpenNodeRenesas.canOpenStack->SDOclient, index, subIndex, 1000, false);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return CO_SDO_AB_GENERAL;
    }

    // upload data
    uint32_t lastLoopTimeUs = micros();
    do {
        uint32_t nowTimeUs         = micros();
        uint32_t timeDifference_us = nowTimeUs - lastLoopTimeUs;
        lastLoopTimeUs             = nowTimeUs;

        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

        SDO_ret = CO_SDOclientUpload(
            canOpenNodeRenesas.canOpenStack->SDOclient,
            timeDifference_us,
            false,
            &abortCode,
            NULL,
            NULL,
            NULL);
        if (abortCode != CO_SDO_AB_NONE) {
            return abortCode;
        }
    } while (SDO_ret > 0);

    // copy data to the user buffer (for long data function must be called
    // several times inside the loop)
    *readSize = CO_SDOclientUploadBufRead(canOpenNodeRenesas.canOpenStack->SDOclient, buf, bufSize);

    return CO_SDO_AB_NONE;
}

CO_SDO_abortCode_t MatrixMCCanOpen::Write_SDO(
    uint8_t devIdx, uint16_t index, uint8_t subIndex, uint8_t* data, size_t dataSize)
{
    static uint8_t lastNodeId = 0xFF;

    uint8_t         nodeId = (uint8_t)DevCanID::Dev1 + devIdx - 0x01;
    CO_SDO_return_t SDO_ret;
    bool_t          bufferPartial = false;

    if (nodeId != lastNodeId) {
        // setup client (this can be skipped, if remote device is the same)
        SDO_ret = CO_SDOclient_setup(
            canOpenNodeRenesas.canOpenStack->SDOclient,
            CO_CAN_ID_SDO_CLI + nodeId,
            CO_CAN_ID_SDO_SRV + nodeId,
            nodeId);
        if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
            return CO_SDO_AB_GENERAL;
        }
        lastNodeId = nodeId;
    }

    // initiate download
    SDO_ret = CO_SDOclientDownloadInitiate(
        canOpenNodeRenesas.canOpenStack->SDOclient, index, subIndex, dataSize, 1000, false);
    if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
        return CO_SDO_AB_GENERAL;
    }

    // fill data
    size_t nWritten =
        CO_SDOclientDownloadBufWrite(canOpenNodeRenesas.canOpenStack->SDOclient, data, dataSize);
    if (nWritten < dataSize) {
        bufferPartial = true;
        // If SDO Fifo buffer is too small, data can be refilled in the loop.
    }

    // download data
    uint32_t lastLoopTimeUs = micros();
    do {
        uint32_t nowTimeUs         = micros();
        uint32_t timeDifference_us = nowTimeUs - lastLoopTimeUs;
        lastLoopTimeUs             = nowTimeUs;

        CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

        SDO_ret = CO_SDOclientDownload(
            canOpenNodeRenesas.canOpenStack->SDOclient,
            timeDifference_us,
            false,
            bufferPartial,
            &abortCode,
            NULL,
            NULL);
        if (abortCode != CO_SDO_AB_NONE) {
            return abortCode;
        }
    } while (SDO_ret > 0);

    return CO_SDO_AB_NONE;
}
