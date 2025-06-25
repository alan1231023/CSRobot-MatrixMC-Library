#include "MatrixMC_R7FA4M1_CAN.h"

#if defined(ARDUINO_MINIMA) || defined(ARDUINO_UNOWIFIR4)

#    include <IRQManager.h>
/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#    define CAN_DEFAULT_MASK (0x1FFFFFFFU)

/**************************************************************************************
 * PROTOTYPE DEFINITIONS
 **************************************************************************************/

extern "C" void can_callback(can_callback_args_t* p_args);
/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

MatrixMC_R7FA4M1_CAN::MatrixMC_R7FA4M1_CAN(int const can_tx_pin, int const can_rx_pin)
    : _can_tx_pin{can_tx_pin}, _can_rx_pin{can_rx_pin}, _is_error{false},
      _err_code{0}, _can_rx_buf{}, _can_bit_timing_cfg{},
      _can_mailbox_mask{CAN_DEFAULT_MASK,
                        CAN_DEFAULT_MASK,
                        CAN_DEFAULT_MASK,
                        CAN_DEFAULT_MASK,
                        0, /* Use no id filtering -> a CAN frame with any ID
                              will be stored in receive mailbox group #0. */
                        CAN_DEFAULT_MASK,
                        0, /* Use no id filtering -> a CAN frame with any ID
                              will be stored in receive mailbox group #2. */
                        CAN_DEFAULT_MASK},
      _can_mailbox{
          /* Mailbox Group #0 */
          {.mailbox_id = 0,
           .id_mode = CAN_ID_MODE_EXTENDED,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_TRANSMIT},
          {.mailbox_id = 1,
           .id_mode = CAN_ID_MODE_EXTENDED,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_TRANSMIT},
          {.mailbox_id = 2,
           .id_mode = CAN_ID_MODE_EXTENDED,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_TRANSMIT},
          {.mailbox_id = 3,
           .id_mode = CAN_ID_MODE_EXTENDED,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_TRANSMIT},
          /* Mailbox Group #1 */
          {.mailbox_id = 4,
           .id_mode = CAN_ID_MODE_EXTENDED,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_TRANSMIT},
          {.mailbox_id = 5,
           .id_mode = CAN_ID_MODE_EXTENDED,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_TRANSMIT},
          {.mailbox_id = 6,
           .id_mode = CAN_ID_MODE_EXTENDED,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_TRANSMIT},
          {.mailbox_id = 7,
           .id_mode = CAN_ID_MODE_EXTENDED,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_TRANSMIT},
          /* Mailbox Group #2 */
          {.mailbox_id = 8,
           .id_mode = CAN_ID_MODE_STANDARD,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_TRANSMIT},
          {.mailbox_id = 9,
           .id_mode = CAN_ID_MODE_STANDARD,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_TRANSMIT},
          {.mailbox_id = 10,
           .id_mode = CAN_ID_MODE_STANDARD,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_TRANSMIT},
          {.mailbox_id = 11,
           .id_mode = CAN_ID_MODE_STANDARD,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_TRANSMIT},
          /* Mailbox Group #3 */
          {.mailbox_id = 12,
           .id_mode = CAN_ID_MODE_STANDARD,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_TRANSMIT},
          {.mailbox_id = 13,
           .id_mode = CAN_ID_MODE_STANDARD,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_TRANSMIT},
          {.mailbox_id = 14,
           .id_mode = CAN_ID_MODE_STANDARD,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_TRANSMIT},
          {.mailbox_id = 15,
           .id_mode = CAN_ID_MODE_STANDARD,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_TRANSMIT},
          /* We only use the very first receive mailbox for receiving. */
          /* Mailbox Group #4 */
          {.mailbox_id = 0,
           .id_mode = CAN_ID_MODE_EXTENDED,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_RECEIVE},
          {.mailbox_id = 1,
           .id_mode = CAN_ID_MODE_EXTENDED,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_RECEIVE},
          {.mailbox_id = 2,
           .id_mode = CAN_ID_MODE_EXTENDED,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_RECEIVE},
          {.mailbox_id = 3,
           .id_mode = CAN_ID_MODE_EXTENDED,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_RECEIVE},
          /* Mailbox Group #5 */
          {.mailbox_id = 4,
           .id_mode = CAN_ID_MODE_EXTENDED,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_RECEIVE},
          {.mailbox_id = 5,
           .id_mode = CAN_ID_MODE_EXTENDED,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_RECEIVE},
          {.mailbox_id = 6,
           .id_mode = CAN_ID_MODE_EXTENDED,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_RECEIVE},
          {.mailbox_id = 7,
           .id_mode = CAN_ID_MODE_EXTENDED,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_RECEIVE},
          /* Mailbox Group #6 */
          {.mailbox_id = 8,
           .id_mode = CAN_ID_MODE_STANDARD,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_RECEIVE},
          {.mailbox_id = 9,
           .id_mode = CAN_ID_MODE_STANDARD,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_RECEIVE},
          {.mailbox_id = 10,
           .id_mode = CAN_ID_MODE_STANDARD,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_RECEIVE},
          {.mailbox_id = 11,
           .id_mode = CAN_ID_MODE_STANDARD,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_RECEIVE},
          /* Mailbox Group #7 */
          {.mailbox_id = 12,
           .id_mode = CAN_ID_MODE_STANDARD,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_RECEIVE},
          {.mailbox_id = 13,
           .id_mode = CAN_ID_MODE_STANDARD,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_RECEIVE},
          {.mailbox_id = 14,
           .id_mode = CAN_ID_MODE_STANDARD,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_RECEIVE},
          {.mailbox_id = 15,
           .id_mode = CAN_ID_MODE_STANDARD,
           .frame_type = CAN_FRAME_TYPE_DATA,
           .mailbox_type = CAN_MAILBOX_RECEIVE}},
      _can_extended_cfg{
          .clock_source = CAN_CLOCK_SOURCE_PCLKB,
          .p_mailbox_mask = _can_mailbox_mask,
          .p_mailbox = _can_mailbox,
          .global_id_mode = CAN_GLOBAL_ID_MODE_MIXED,
          .mailbox_count = CAN_MAX_NO_MAILBOXES,
          .message_mode = CAN_MESSAGE_MODE_OVERWRITE,
          .p_fifo_int_cfg = nullptr,
          .p_rx_fifo_cfg = nullptr,
      },
      _can_cfg{
          .channel = 0,
          .p_bit_timing = &_can_bit_timing_cfg,
          .p_callback = can_callback,
          .p_context = this,
          .p_extend = &_can_extended_cfg,
          .ipl = (12),
          .error_irq = FSP_INVALID_VECTOR,
          .rx_irq = FSP_INVALID_VECTOR,
          .tx_irq = FSP_INVALID_VECTOR,
      }
{}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

bool MatrixMC_R7FA4M1_CAN::begin(CanBitRate const can_bitrate)
{
    return begin(static_cast<uint32_t>(can_bitrate));
}

bool MatrixMC_R7FA4M1_CAN::begin(uint32_t const can_bitrate)
{
    bool init_ok = true;

    /* Configure the pins for CAN.
     */
    int const max_index = PINS_COUNT;
    init_ok &= cfg_pins(max_index, _can_tx_pin, _can_rx_pin);

    /* Configure the interrupts.
     */
    CanIrqReq_t irq_req{
        .ctrl = &_can_ctrl,
        .cfg  = &_can_cfg,
    };
    init_ok &= IRQManager::getInstance().addPeripheral(IRQ_CAN, &irq_req);

    /* Calculate the CAN bitrate based on the value of this functions parameter.
     */
    static uint32_t const F_CAN_CLK_Hz = 24 * 1000 * 1000UL;
    static uint32_t const TQ_MIN       = 8;
    static uint32_t const TQ_MAX       = 25;
    static uint32_t const TSEG_1_MIN   = 4;
    static uint32_t const TSEG_1_MAX   = 16;
    static uint32_t const TSEG_2_MIN   = 2;
    static uint32_t const TSEG_2_MAX   = 8;

    auto [is_valid_baudrate, baud_rate_prescaler, time_segment_1, time_segment_2] =
        calc_can_bit_timing(
            can_bitrate,
            F_CAN_CLK_Hz,
            TQ_MIN,
            TQ_MAX,
            TSEG_1_MIN,
            TSEG_1_MAX,
            TSEG_2_MIN,
            TSEG_2_MAX);
    init_ok &= is_valid_baudrate;

    if (is_valid_baudrate) {
        _can_bit_timing_cfg.baud_rate_prescaler        = baud_rate_prescaler;
        _can_bit_timing_cfg.time_segment_1             = time_segment_1;
        _can_bit_timing_cfg.time_segment_2             = time_segment_2;
        _can_bit_timing_cfg.synchronization_jump_width = 1;
    }

    /* Initialize the peripheral's FSP driver. */
    if (R_CAN_Open(&_can_ctrl, &_can_cfg) != FSP_SUCCESS) init_ok = false;

    return init_ok;
}

void MatrixMC_R7FA4M1_CAN::end()
{
    R_CAN_Close(&_can_ctrl);
}

void MatrixMC_R7FA4M1_CAN::setFilterMask_Standard(uint32_t const mask)
{
    _can_mailbox_mask[6] = mask;
    _can_mailbox_mask[7] = mask;
}

void MatrixMC_R7FA4M1_CAN::setFilterMask_Extended(uint32_t const mask)
{
    _can_mailbox_mask[4] = mask;
    _can_mailbox_mask[5] = mask;
}

void MatrixMC_R7FA4M1_CAN::setFilterId_Standard(size_t const mailbox, uint32_t const id)
{
    if (mailbox > CAN_MAX_NO_STANDARD_MAILBOXES) return;

    size_t const mailbox_idx             = CAN_MAX_STANDARD_MAILBOX_OFFSET + mailbox;
    _can_mailbox[mailbox_idx].mailbox_id = id;
}

void MatrixMC_R7FA4M1_CAN::setFilterId_Extended(size_t const mailbox, uint32_t const id)
{
    if (mailbox > CAN_MAX_NO_EXTENDED_MAILBOXES) return;

    size_t const mailbox_idx             = CAN_MAX_EXTENDED_MAILBOX_OFFSET + mailbox;
    _can_mailbox[mailbox_idx].mailbox_id = id;
}

int MatrixMC_R7FA4M1_CAN::enableInternalLoopback()
{
    if (fsp_err_t const rc = R_CAN_ModeTransition(
            &_can_ctrl, CAN_OPERATION_MODE_NORMAL, CAN_TEST_MODE_LOOPBACK_EXTERNAL);
        rc != FSP_SUCCESS)
        return -rc;

    return 1;
}

int MatrixMC_R7FA4M1_CAN::disableInternalLoopback()
{
    if (fsp_err_t const rc =
            R_CAN_ModeTransition(&_can_ctrl, CAN_OPERATION_MODE_NORMAL, CAN_TEST_MODE_DISABLED);
        rc != FSP_SUCCESS)
        return -rc;

    return 1;
}

int MatrixMC_R7FA4M1_CAN::write(CanMsg const& msg)
{
    bool const is_standard_id = msg.isStandardId();

    can_frame_t can_msg = {
        /* id               = */ is_standard_id ? msg.getStandardId() : msg.getExtendedId(),
        /* id_mode          = */
        is_standard_id ? CAN_ID_MODE_STANDARD : CAN_ID_MODE_EXTENDED,
        /* type             = */ CAN_FRAME_TYPE_DATA,
        /* data_length_code = */ min(msg.data_length, CAN_DATA_BUFFER_LENGTH),
        /* options          = */ 0};

    memcpy(can_msg.data, msg.data, can_msg.data_length_code);

    if (fsp_err_t const rc = R_CAN_Write(
            &_can_ctrl, is_standard_id ? CAN_MAILBOX_ID_0 : CAN_MAILBOX_ID_16, &can_msg);
        rc != FSP_SUCCESS)
        return -rc;

    return 1;
}

size_t MatrixMC_R7FA4M1_CAN::available()
{
    return _can_rx_buf.available();
}

CanMsg MatrixMC_R7FA4M1_CAN::read()
{
    return _can_rx_buf.dequeue();
}

void MatrixMC_R7FA4M1_CAN::onCanCallback(can_callback_args_t* p_args)
{
    switch (p_args->event) {
    case CAN_EVENT_TX_COMPLETE:
        // if (eventTXCallback) {
        //     eventTXCallback(p_args);
        // }
        break;
    case CAN_EVENT_RX_COMPLETE:   // Currently driver don't support this. This is
                                  // unreachable code for now.
    {
        /* Extract the received CAN message. */
        // CanMsg const msg(
        //     (p_args->frame.id_mode == CAN_ID_MODE_STANDARD) ? CanStandardId(p_args->frame.id)
        //                                                     : CanExtendedId(p_args->frame.id),
        //     p_args->frame.data_length_code,
        //     p_args->frame.data);
        // /* Store the received CAN message in the receive buffer. */
        // _can_rx_buf.enqueue(msg);

        if (eventRXCallback) {
            eventRXCallback(p_args);
        }
    } break;
    case CAN_EVENT_ERR_WARNING: /* error warning event */ errorState |= CAN_EVENT_ERR_WARNING;
    case CAN_EVENT_ERR_PASSIVE: /* error passive event */ errorState |= CAN_EVENT_ERR_PASSIVE;
    case CAN_EVENT_ERR_BUS_OFF: /* error bus off event */ errorState |= CAN_EVENT_ERR_BUS_OFF;
    case CAN_EVENT_BUS_RECOVERY: /* Bus recovery error event */
        errorState |= CAN_EVENT_BUS_RECOVERY;
    case CAN_EVENT_MAILBOX_MESSAGE_LOST: /* overwrite/overrun error event */
        errorState |= CAN_EVENT_MAILBOX_MESSAGE_LOST;
    case CAN_EVENT_ERR_BUS_LOCK: /* Bus lock detected (32 consecutive dominant
                                    bits). */
        errorState |= CAN_EVENT_ERR_BUS_LOCK;

    case CAN_EVENT_ERR_CHANNEL: /* Channel error has occurred. */
        errorState |= CAN_EVENT_ERR_CHANNEL;
    case CAN_EVENT_TX_ABORTED: /* Transmit abort event. */ errorState |= CAN_EVENT_TX_ABORTED;
    case CAN_EVENT_ERR_GLOBAL: /* Global error has occurred. */ errorState |= CAN_EVENT_ERR_GLOBAL;
    case CAN_EVENT_TX_FIFO_EMPTY: /* Transmit FIFO is empty. */
    {
        errorState |= CAN_EVENT_TX_FIFO_EMPTY;
        _is_error = true;
        _err_code = p_args->event;
    } break;
    }
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

bool MatrixMC_R7FA4M1_CAN::cfg_pins(int const max_index, int const can_tx_pin, int const can_rx_pin)
{
    /* Verify if indices are good. */
    if (can_tx_pin < 0 || can_rx_pin < 0 || can_tx_pin >= max_index || can_rx_pin >= max_index) {
        return false;
    }

    /* Getting configuration from table. */
    const uint16_t* cfg        = nullptr;
    auto            cfg_can_tx = getPinCfgs(can_tx_pin, PIN_CFG_REQ_CAN_TX);
    auto            cfg_can_rx = getPinCfgs(can_rx_pin, PIN_CFG_REQ_CAN_RX);

    /* Verify if configurations are good. */
    if (cfg_can_tx[0] == 0 || cfg_can_rx[0] == 0) {
        return false;
    }

    /* Verify if channel is the same for all pins. */
    uint32_t const ch_can_tx = GET_CHANNEL(cfg_can_tx[0]);
    uint32_t const ch_can_rx = GET_CHANNEL(cfg_can_rx[0]);
    if (ch_can_tx != ch_can_rx) {
        return false;
    }

    /* Actually configure pin functions. */
    R_IOPORT_PinCfg(
        &g_ioport_ctrl,
        g_pin_cfg[can_tx_pin].pin,
        (uint32_t)(IOPORT_CFG_PERIPHERAL_PIN) | (uint32_t)(IOPORT_PERIPHERAL_CAN));
    R_IOPORT_PinCfg(
        &g_ioport_ctrl,
        g_pin_cfg[can_rx_pin].pin,
        (uint32_t)(IOPORT_CFG_PERIPHERAL_PIN) | (uint32_t)(IOPORT_PERIPHERAL_CAN));

    return true;
}

std::tuple<bool, uint32_t, uint32_t, uint32_t> MatrixMC_R7FA4M1_CAN::calc_can_bit_timing(
    uint32_t const can_bitrate, uint32_t const can_clock_Hz, uint32_t const tq_min,
    uint32_t const tq_max, uint32_t const tseg1_min, uint32_t const tseg1_max,
    uint32_t const tseg2_min, uint32_t const tseg2_max)
{
    /* Note: Concerning the calculation of
     *   - baud_rate_prescaler
     *   - time_segment_1 (TSEG1)
     *   - time_segment_2 (TSEG2)
     * also compare with section 30.4.3, RA4M1 Group User Manual, Rev. 1.00, October 2019.
     */
    for (uint32_t tq = tq_max; tq >= tq_min; tq--) {
        /* Determine the CAN baud rate prescaler. */
        double const brp =
            static_cast<double>(can_clock_Hz) / (tq * static_cast<double>(can_bitrate));
        /* Extract the sub-comma part of the baud rate prescaler. */
        double       brp_ipart = 0.0f;
        double const brp_fract = modf(brp, &brp_ipart);
        /* If the fractional part is sufficiently close to zero, we have
         * found a valid prescaler configuration.
         */
        if (brp_fract < 0.01) {
            uint32_t const baud_rate_prescaler = static_cast<uint32_t>(brp_ipart);
            /* Assign TSEG1 and TSEG2 to set the sample point at 75%. */
            uint32_t const time_segment_1 = static_cast<int>(static_cast<float>(tq) * 0.75f) - 1;
            uint32_t const time_segment_2 = tq - time_segment_1 - 1;
            /* Check if the found values are within the allowed boundary. */
            if (time_segment_1 < tseg1_min || time_segment_1 > tseg1_max) continue;
            if (time_segment_2 < tseg2_min || time_segment_2 > tseg2_max) continue;
            /* We've found a valid configuration, exit here. */
            return std::make_tuple(true, baud_rate_prescaler, time_segment_1, time_segment_2);
        }
    }

    return std::make_tuple(false, 0, 0, 0);
}

/**************************************************************************************
 * CALLBACKS FOR FSP FRAMEWORK
 **************************************************************************************/

extern "C" void can_callback(can_callback_args_t* p_args)
{
    MatrixMC_R7FA4M1_CAN* this_ptr = (MatrixMC_R7FA4M1_CAN*)(p_args->p_context);
    this_ptr->onCanCallback(p_args);
}

#endif /* defined(ARDUINO_MINIMA) || defined(ARDUINO_UNOWIFIR4) */

/**************************************************************************************
 * OBJECT INSTANTIATION
 **************************************************************************************/

#if CAN_HOWMANY > 0
MatrixMC_R7FA4M1_CAN MatrixMC_CAN(PIN_CAN0_TX, PIN_CAN0_RX);
#endif

#if CAN_HOWMANY > 1
MatrixMC_R7FA4M1_CAN MatrixMC_CAN1(PIN_CAN1_TX, PIN_CAN1_RX);
#endif
