#include "../CANopenNode/301/CO_driver.h"
#include "CO_app_STM32.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>


/* Local CAN module object */
static CO_CANmodule_t* CANModule_local = NULL; /* Local instance of global CAN module */

/* CAN masks for identifiers */
#define CANID_MASK 0x07FF /*!< CAN standard ID mask */
#define FLAG_RTR   0x8000 /*!< RTR flag, part of identifier */

void TxMailboxCompleteCallback(can_callback_args_t* p_args);
void RxFifoMsgPendingCallback(can_callback_args_t* p_args);

/******************************************************************************/
void CO_CANsetConfigurationMode(void* CANptr)
{
    /* Put CAN module in configuration mode */
    if (CANptr != NULL) {
#ifdef CO_STM32_FDCAN_Driver
#else
        // ((CANopenNodeRenesas*)CANptr)->CANHandle->end();
#endif
    }
}

/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t* CANmodule)
{
    /* Put CAN module in normal mode */
    if (CANmodule->CANptr != NULL) {
        CANopenNodeRenesas* CANopenNode = (CANopenNodeRenesas*)CANmodule->CANptr;
        CanBitRate          bitRate     = CanBitRate::BR_250k;
        switch (CANopenNode->baudrate) {
        case 125: bitRate = CanBitRate::BR_125k; break;
        case 250: bitRate = CanBitRate::BR_250k; break;
        case 500: bitRate = CanBitRate::BR_500k; break;
        case 1000: bitRate = CanBitRate::BR_1000k; break;
        default: break;
        }
        if (CANopenNode->CANHandle->begin(bitRate)) {
            CANopenNode->CANHandle->eventTXCallback = TxMailboxCompleteCallback;
            CANopenNode->CANHandle->eventRXCallback = RxFifoMsgPendingCallback;
            CANmodule->CANnormal                    = true;
        }
    }
}

/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
    CO_CANmodule_t* CANmodule, void* CANptr, CO_CANrx_t rxArray[], uint16_t rxSize,
    CO_CANtx_t txArray[], uint16_t txSize, uint16_t CANbitRate)
{

    /* verify arguments */
    if (CANmodule == NULL || rxArray == NULL || txArray == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Hold CANModule variable */
    CANmodule->CANptr = CANptr;

    /* Keep a local copy of CANModule */
    CANModule_local = CANmodule;

    /* Configure object variables */
    CANmodule->rxArray           = rxArray;
    CANmodule->rxSize            = rxSize;
    CANmodule->txArray           = txArray;
    CANmodule->txSize            = txSize;
    CANmodule->CANerrorStatus    = 0;
    CANmodule->CANnormal         = false;
    CANmodule->useCANrxFilters   = false; /* Do not use HW filters */
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount        = 0U;
    CANmodule->errOld            = 0U;

    /* Reset all variables */
    for (uint16_t i = 0U; i < rxSize; i++) {
        rxArray[i].ident          = 0U;
        rxArray[i].mask           = 0xFFFFU;
        rxArray[i].object         = NULL;
        rxArray[i].CANrx_callback = NULL;
    }
    for (uint16_t i = 0U; i < txSize; i++) {
        txArray[i].bufferFull = false;
    }

    /***************************************/
    /* STM32 related configuration */
    /***************************************/
    ((CANopenNodeRenesas*)CANptr)->HWInitFunction();

    /*
     * Configure global filter that is used as last check if message did not pass any of other
     * filters:
     *
     * We do not rely on hardware filters in this example
     * and are performing software filters instead
     *
     * Accept non-matching standard ID messages
     * Reject non-matching extended ID messages
     */

    //     CAN_FilterTypeDef FilterConfig;
    // #if defined(CAN)
    //     FilterConfig.FilterBank = 0;
    // #else
    //     if (((CAN_HandleTypeDef*)((CANopenNodeRenesas*)CANmodule->CANptr)->CANHandle)->Instance
    //     == CAN1) {
    //         FilterConfig.FilterBank = 0;
    //     } else {
    //         FilterConfig.FilterBank = 14;
    //     }
    // #endif
    //     FilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    //     FilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    //     FilterConfig.FilterIdHigh         = 0x0;
    //     FilterConfig.FilterIdLow          = 0x0;
    //     FilterConfig.FilterMaskIdHigh     = 0x0;
    //     FilterConfig.FilterMaskIdLow      = 0x0;
    //     FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    //     FilterConfig.FilterActivation     = ENABLE;
    //     FilterConfig.SlaveStartFilterBank = 14;

    //     if (HAL_CAN_ConfigFilter(((CANopenNodeRenesas*)CANptr)->CANHandle, &FilterConfig) !=
    //     HAL_OK) {
    //         return CO_ERROR_ILLEGAL_ARGUMENT;
    //     }
    // #endif

    return CO_ERROR_NO;
}

/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t* CANmodule)
{
    if (CANmodule != NULL && CANmodule->CANptr != NULL) {
        ((CANopenNodeRenesas*)CANmodule->CANptr)->CANHandle->end();
    }
}

/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
    CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, uint16_t mask, bool_t rtr,
    void* object, void (*CANrx_callback)(void* object, void* message))
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    if (CANmodule != NULL && object != NULL && CANrx_callback != NULL &&
        index < CANmodule->rxSize) {
        CO_CANrx_t* buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object         = object;
        buffer->CANrx_callback = CANrx_callback;

        /*
         * Configure global identifier, including RTR bit
         *
         * This is later used for RX operation match case
         */
        buffer->ident = (ident & CANID_MASK) | (rtr ? FLAG_RTR : 0x00);
        buffer->mask  = (mask & CANID_MASK) | FLAG_RTR;

        /* Set CAN hardware module filter and mask. */
        if (CANmodule->useCANrxFilters) {
            __NOP();
        }
    } else {
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}

/******************************************************************************/
CO_CANtx_t* CO_CANtxBufferInit(
    CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, bool_t rtr, uint8_t noOfBytes,
    bool_t syncFlag)
{
    CO_CANtx_t* buffer = NULL;

    if (CANmodule != NULL && index < CANmodule->txSize) {
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer */
        buffer->ident      = ((uint32_t)ident & CANID_MASK) | ((uint32_t)(rtr ? FLAG_RTR : 0x00));
        buffer->DLC        = noOfBytes;
        buffer->bufferFull = false;
        buffer->syncFlag   = syncFlag;
    }
    return buffer;
}

/**
 * \brief           Send CAN message to network
 * This function must be called with atomic access.
 *
 * \param[in]       CANmodule: CAN module instance
 * \param[in]       buffer: Pointer to buffer to transmit
 */
static uint8_t prv_send_can_message(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer)
{
    CANopenNodeRenesas* CANopenNode = (CANopenNodeRenesas*)CANmodule->CANptr;

    uint8_t      success = 0;
    CanMsg const msg(CanStandardId(buffer->ident), buffer->DLC, buffer->data);
    success = CANopenNode->CANHandle->write(msg) > 0;
    return success;
}

/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer)
{
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if (buffer->bufferFull) {
        if (!CANmodule->firstCANtxMessage) {
            /* don't set error, if bootup message is still on buffers */
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    /*
     * Send message to CAN network
     *
     * Lock interrupts for atomic operation
     */
    CO_LOCK_CAN_SEND(CANmodule);
    if (prv_send_can_message(CANmodule, buffer)) {
        CANmodule->bufferInhibitFlag = buffer->syncFlag;
    } else {
        buffer->bufferFull = true;
        CANmodule->CANtxCount++;
    }
    CO_UNLOCK_CAN_SEND(CANmodule);
    return err;
}

/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t* CANmodule)
{
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if (/*messageIsOnCanBuffer && */ CANmodule->bufferInhibitFlag) {
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted                  = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if (CANmodule->CANtxCount > 0) {
        for (uint16_t i = CANmodule->txSize; i > 0U; --i) {
            if (CANmodule->txArray[i].bufferFull) {
                if (CANmodule->txArray[i].syncFlag) {
                    CANmodule->txArray[i].bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);
    if (tpdoDeleted) {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}

/******************************************************************************/
/* Get error counters from the module. If necessary, function may use
 * different way to determine errors. */
static uint16_t rxErrors = 0, txErrors = 0, overflow = 0;

void CO_CANmodule_process(CO_CANmodule_t* CANmodule)
{
    CANopenNodeRenesas* CANopenNode = (CANopenNodeRenesas*)CANmodule->CANptr;
    uint32_t            err         = CANopenNode->CANHandle->errorState;

    if (CANmodule->errOld != err) {
        uint16_t status   = CANmodule->CANerrorStatus;
        CANmodule->errOld = err;

        if (err & CAN_EVENT_ERR_BUS_OFF) {
            status |= CO_CAN_ERRTX_BUS_OFF;
        } else {
            status &= 0xFFFF ^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE |
                                CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE);
            if (err & CAN_EVENT_ERR_WARNING) {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRTX_WARNING;
            }
            if (err & CAN_EVENT_ERR_PASSIVE) {
                status |= CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_PASSIVE;
            }
        }
        CANmodule->CANerrorStatus = status;
    }
}

/**
 * \brief           Read message from RX FIFO
 * \param           hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       fifo: Fifo number to use for read
 * \param[in]       fifo_isrs: List of interrupts for respected FIFO
 */

static void prv_read_can_received_msg(can_callback_args_t* p_args)
{

    CO_CANrxMsg_t rcvMsg;
    CO_CANrx_t*   buffer = NULL; /* receive message buffer from CO_CANmodule_t object. */
    uint16_t      index;         /* index of received message */
    uint32_t      rcvMsgIdent;   /* identifier of the received message */
    uint8_t       messageFound = 0;


    // static CAN_RxHeaderTypeDef rx_hdr;
    /* Read received message from FIFO */
    memcpy(rcvMsg.data, p_args->frame.data, p_args->frame.data_length_code);

    /* Setup identifier (with RTR) and length */
    rcvMsg.ident =
        p_args->frame.id | (p_args->frame.type == CAN_FRAME_TYPE_REMOTE ? FLAG_RTR : 0x00);
    rcvMsg.dlc  = p_args->frame.data_length_code;
    rcvMsgIdent = rcvMsg.ident;

    /*
     * Hardware filters are not used for the moment
     * \todo: Implement hardware filters...
     */
    if (CANModule_local->useCANrxFilters) {
        __BKPT(0);
    } else {
        /*
         * We are not using hardware filters, hence it is necessary
         * to manually match received message ID with all buffers
         */
        buffer = CANModule_local->rxArray;
        for (index = CANModule_local->rxSize; index > 0U; --index, ++buffer) {
            if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U) {
                messageFound = 1;
                break;
            }
        }
    }

    /* Call specific function, which will process the message */
    if (messageFound && buffer != NULL && buffer->CANrx_callback != NULL) {
        buffer->CANrx_callback(buffer->object, (void*)&rcvMsg);
    }
}

/**
 * \brief           Rx FIFO 0 callback.
 * \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 */
void RxFifoMsgPendingCallback(can_callback_args_t* p_args)
{
    prv_read_can_received_msg(p_args);
}

/**
 * \brief           TX buffer has been well transmitted callback
 * \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 * \param[in]       MailboxNumber: the mailbox number that has been transmitted
 */
void CO_CANinterrupt_TX(CO_CANmodule_t* CANmodule, uint32_t MailboxNumber)
{

    CANmodule->firstCANtxMessage = false; /* First CAN message (bootup) was sent successfully */
    CANmodule->bufferInhibitFlag = false; /* Clear flag from previous message */
    if (CANmodule->CANtxCount > 0U) {     /* Are there any new messages waiting to be send */
        CO_CANtx_t* buffer = &CANmodule->txArray[0]; /* Start with first buffer handle */
        uint16_t    i;

        /*
         * Try to send more buffers, process all empty ones
         *
         * This function is always called from interrupt,
         * however to make sure no preemption can happen, interrupts are anyway locked
         * (unless you can guarantee no higher priority interrupt will try to access to CAN instance
         * and send data, then no need to lock interrupts..)
         */
        CO_LOCK_CAN_SEND(CANmodule);
        for (i = CANmodule->txSize; i > 0U; --i, ++buffer) {
            /* Try to send message */
            if (buffer->bufferFull) {
                if (prv_send_can_message(CANmodule, buffer)) {
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    CANmodule->bufferInhibitFlag = buffer->syncFlag;
                }
            }
        }
        /* Clear counter if no more messages */
        if (i == 0U) {
            CANmodule->CANtxCount = 0U;
        }
        CO_UNLOCK_CAN_SEND(CANmodule);
    }
}

void TxMailboxCompleteCallback(can_callback_args_t* p_args)
{
    CO_CANinterrupt_TX(CANModule_local, p_args->mailbox);
}
