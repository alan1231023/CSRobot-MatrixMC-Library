#ifndef MatrixMC_R7FA4M1_CAN_H
#define MatrixMC_R7FA4M1_CAN_H

#include <Arduino.h>
#if defined(ARDUINO_MINIMA) || defined(ARDUINO_UNOWIFIR4)
#    include "api/HardwareCAN.h"
#    include "bsp_api.h"
#    include "r_can.h"
#    include "sync.h"
#    include <cstdint>
#    include <math.h> /* modf */
#    include <tuple>



typedef CanMsg CanMsg;
typedef void (*MatrixMCCanCallbackFunction)(can_callback_args_t* p_args);

class SyncCanMsgRingbuffer
{
public:
    SyncCanMsgRingbuffer()
        : _can_msg_buf{}
    {}


    bool isFull() const
    {
        synchronized
        {
            _can_msg_buf.isFull();
        }
    }
    void enqueue(CanMsg const& msg)
    {
        synchronized
        {
            _can_msg_buf.enqueue(msg);
        }
    }

    bool isEmpty() const
    {
        synchronized
        {
            return _can_msg_buf.isEmpty();
        }
    }
    CanMsg dequeue()
    {
        synchronized
        {
            return _can_msg_buf.dequeue();
        }
    }

    size_t available() const
    {
        synchronized
        {
            return _can_msg_buf.available();
        }
    }

private:
    CanMsgRingbuffer _can_msg_buf;
};

class MatrixMC_R7FA4M1_CAN final : public HardwareCAN
{

public:
    MatrixMC_R7FA4M1_CAN(int const can_tx_pin, int const can_rx_pin);
    virtual ~MatrixMC_R7FA4M1_CAN() {}

    bool begin(CanBitRate const can_bitrate) override;
    bool begin(uint32_t const can_bitrate);
    void end() override;

    void setFilterMask_Standard(uint32_t const mask);
    void setFilterMask_Extended(uint32_t const mask);
    void setFilterId_Standard(size_t const mailbox, uint32_t const id);
    void setFilterId_Extended(size_t const mailbox, uint32_t const id);

    int enableInternalLoopback();
    int disableInternalLoopback();

    int    write(CanMsg const& msg) override;
    size_t available() override;
    CanMsg read() override;

    inline bool isError(int& err_code) const
    {
        err_code = _err_code;
        return _is_error;
    }
    inline void clearError()
    {
        _is_error = false;
        _err_code = 0;
    }

    uint32_t errorState;

    /* This function is used by the library and should NOT be called by the
     * user.
     */
    void                        onCanCallback(can_callback_args_t* p_args);
    MatrixMCCanCallbackFunction eventTXCallback;
    MatrixMCCanCallbackFunction eventRXCallback;

    static size_t constexpr CAN_MAX_NO_STANDARD_MAILBOXES = 8U;
    static size_t constexpr CAN_MAX_NO_EXTENDED_MAILBOXES = 8U;

private:
    static size_t constexpr CAN_MAX_NO_MAILBOXES            = 32U;
    static size_t constexpr CAN_MAX_STANDARD_MAILBOX_OFFSET = 24U;
    static size_t constexpr CAN_MAX_EXTENDED_MAILBOX_OFFSET = 16U;

    int const            _can_tx_pin;
    int const            _can_rx_pin;
    bool                 _is_error;
    int                  _err_code;
    SyncCanMsgRingbuffer _can_rx_buf;

    can_instance_ctrl_t  _can_ctrl;
    can_bit_timing_cfg_t _can_bit_timing_cfg;
    uint32_t             _can_mailbox_mask[CAN_MAX_NO_MAILBOXES / 4];
    can_mailbox_t        _can_mailbox[CAN_MAX_NO_MAILBOXES];
    can_extended_cfg_t   _can_extended_cfg;
    can_cfg_t            _can_cfg;

    static bool cfg_pins(int const max_index, int const can_tx_pin, int const can_rx_pin);

    static std::tuple<
        bool,     /* valid result found */
        uint32_t, /* baud_rate_prescaler */
        uint32_t, /* time_segment_1 */
        uint32_t> /* time_segment_2 */
    calc_can_bit_timing(
        uint32_t const can_bitrate, uint32_t const can_clock_Hz, uint32_t const tq_min,
        uint32_t const tq_max, uint32_t const tseg1_min, uint32_t const tseg1_max,
        uint32_t const tseg2_min, uint32_t const tseg2_max);
};

#endif /* defined(ARDUINO_MINIMA) || defined(ARDUINO_UNOWIFIR4) */


#if CAN_HOWMANY > 0
extern MatrixMC_R7FA4M1_CAN MatrixMC_CAN;
#endif

#if CAN_HOWMANY > 1
extern MatrixMC_R7FA4M1_CAN MatrixMC_CAN1;
#endif

#endif   // MatrixMC_R7FA4M1_CAN_H