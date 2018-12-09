/*
 * @brief LPC5460X CAN driver
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2016
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * NXP products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#ifndef __CAN_5461X_H_
#define __CAN_5461X_H_

#include "fsl_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup CAN_5461X CHIP:  LPC5460X CAN driver
 * @ingroup CHIP_5461X_DRIVERS
 * @{
 */

//#ifdef LPC54618_SERIES
#ifndef USE_FD
#define USE_FD
#endif
//#endif

#ifdef USE_FD
#ifndef CAN_PSR_DLEC_MASK
#define CAN_PSR_DLEC_MASK                        (0x700U)
#endif
#ifndef CAN_PSR_RESI_MASK
#define CAN_PSR_RESI_MASK                        (0x800U)
#endif
#ifndef CAN_PSR_RBRS_MASK
#define CAN_PSR_RBRS_MASK                        (0x1000U)
#endif
#ifndef CAN_PSR_RFDF_MASK
#define CAN_PSR_RFDF_MASK                        (0x2000U)
#endif
#ifndef CAN_CCCR_BRSE_MASK
#define CAN_CCCR_BRSE_MASK                       (0x200U)
#endif
#ifndef CAN_CCCR_FDOE_MASK
#define CAN_CCCR_FDOE_MASK                       (0x100U)
#endif
#ifndef CAN_CCCR_NISO_MASK
#define CAN_CCCR_NISO_MASK                       (0x8000U)
#endif
#endif  

#ifdef USE_FD
/*! @brief Maximum size of a CAN frame. Must be a valid CAN FD value */
#define CAN_MAX_MESSAGE_BYTES 64
#else
/*! @brief Maximum size of a CAN frame. Must be a valid CAN value */
#define CAN_MAX_MESSAGE_BYTES 8
#endif  
#define CAN_MAX_MESSAGE_WORDS (CAN_MAX_MESSAGE_BYTES >> 2)

/*! @brief Number of standard filters (0 - 128) */
#define CAN_NUM_STANDARD_FILTERS   128
/*! @brief Number of extended filters (0 - 64) */
#define CAN_NUM_EXTENDED_FILTERS   64
/*! Number of receive FIFOs (1 - 2) */
#define CAN_NUM_RX_FIFOS 2
/*! @brief Number of transmit message buffers (1 - 32) */
#define CAN_NUM_TX_MESSAGE_BUFFERS 32
/*! @brief Number of receive message buffers. Cannot be changed */
#define CAN_NUM_RX_MESSAGE_BUFFERS 64

/*! @brief Receive message buffer helper macro */
#define CAN_RX_MB_STD(id, mbIdx) \
    (7UL << 27) | ((id & 0x7FF) << 16) | (mbIdx & 0x3F)

/*! @brief Receive message buffer extended helper macro - low */
#define CAN_RX_MB_EXT_LOW(id, mbIdx) \
    (7UL << 29) | (id & 0x1FFFFFFFUL)
/*! @brief Receive message buffer extended helper macro - high */
#define CAN_RX_MB_EXT_HIGH(id, mbIdx) \
    (mbIdx & 0x3FUL)

/*! @brief CAN Rx FIFO 0 Mask helper macro. */
#define CAN_RX_FIFO0_STD_MASK(match, mask) \
    (2UL << 30) | (1UL << 27) | ((match & 0x7FF) << 16) | (mask & 0x7FF)

/*! @brief CAN Rx FIFO 0 extended Mask helper macro - low. */
#define CAN_RX_FIFO0_EXT_MASK_LOW(match) \
    (1UL << 29) | ((match & 0x1FFFFFFF))
/*! @brief CAN Rx FIFO 0 extended Mask helper macro - high. */
#define CAN_RX_FIFO0_EXT_MASK_HIGH(mask) \
    (2UL << 30) | ((mask & 0x1FFFFFFF))

/*! @brief CAN Rx FIFO 1 Mask helper macro. */
#define CAN_RX_FIFO1_STD_MASK(match, mask) \
    (2UL << 30) | (2UL << 27) | ((match & 0x7FF) << 16) | (mask & 0x7FF)

/*! @brief CAN Rx FIFO 1 extended Mask helper macro - low. */
#define CAN_RX_FIFO1_EXT_MASK_LOW(match) \
    (2UL << 29) | ((match & 0x1FFFFFFF))
/*! @brief CAN Rx FIFO 1 extended Mask helper macro - high. */
#define CAN_RX_FIFO1_EXT_MASK_HIGH(mask) \
    (2UL << 30) | ((mask & 0x1FFFFFFF))

/*! @brief CAN transfer status. */
enum _can_status
{
    kStatus_CAN_TxBusy         = 10,  /*!< Tx Message Buffer is Busy. */
    kStatus_CAN_TxIdle         = 11,  /*!< Tx Message Buffer is Idle. */
    kStatus_CAN_TxSwitchToRx   = 12,  /*!< Remote Message is send out and Message buffer changed to Receive one. */
    kStatus_CAN_RxBusy         = 13,  /*!< Rx Message Buffer is Busy. */
    kStatus_CAN_RxIdle         = 14,  /*!< Rx Message Buffer is Idle. */
    kStatus_CAN_RxOverflow     = 15,  /*!< Rx Message Buffer is Overflowed. */
    kStatus_CAN_RxFifoBusy     = 16,  /*!< Rx Message FIFO is Busy. */
    kStatus_CAN_RxFifoIdle     = 17,  /*!< Rx Message FIFO is Idle. */
    kStatus_CAN_RxFifoOverflow = 18,  /*!< Rx Message FIFO is overflowed. */
    kStatus_CAN_RxFifoWarning  = 19,  /*!< Rx Message FIFO is almost overflowed. */
    kStatus_CAN_ErrorStatus    = 20,  /*!< CAN Module Error and Status. */
    kStatus_CAN_UnHandled      = 21,  /*!< UnHandled Interrupt asserted. */
};

/*! @brief CAN frame format. */
typedef enum _can_frame_format
{
    kCAN_FrameFormatStandard = 0, /*!< Standard frame format attribute. */
    kCAN_FrameFormatExtend   = 1, /*!< Extend frame format attribute. */
} can_frame_format_t;

/*! @brief CAN frame type. */
typedef enum _can_frame_type
{
    kCAN_FrameTypeData   = 0, /*!< Data frame type attribute. */
    kCAN_FrameTypeRemote = 1, /*!< Remote frame type attribute. */
} can_frame_type_t;

/*! @brief CAN protocol type. */
typedef enum _can_proto_type
{
    kCAN_ProtoTypeClassic = 0, /*!< Classic frame format attribute. */
#ifdef USE_FD
    kCAN_ProtoTypeFD      = 1, /*!< FD frame format attribute. */
#endif
} can_proto_type_t;

/*! @brief CAN bitrate mode type. */
typedef enum _can_bitrate_type
{
    kCAN_BitrateModeTypeNoSwitch = 0, /*!< No bitrate switch attribute. */
    kCAN_BitrateModeTypeSwitch   = 1, /*!< Bitrate switch attribute. */
} can_bitratemode_type_t;

/*!
 * @brief CAN status flags.
 *
 * This provides constants for the CAN status flags for use in the CAN functions.
 * Note: The CPU read action clears CAN_ErrorFlag, therefore user need to
 * read CAN_ErrorFlag and distinguish which error is occur using
 * @ref _can_error_flags enumerations.
 */
enum _can_flags
{
    kCAN_LastErrorCodeFlag     = CAN_PSR_LEC_MASK,  /*!< Last error code */
    kCAN_ActivityFlag          = CAN_PSR_ACT_MASK,  /*!< Activity - sync, idle, rx, tx */
    kCAN_ErrorPassiveFlag      = CAN_PSR_EP_MASK,   /*!< In error passive state */
    kCAN_ErrorWarningFlag      = CAN_PSR_EW_MASK,   /*!< In error warning state */
    kCAN_BusOffFlag            = CAN_PSR_BO_MASK,   /*!< In bus off state */
#ifdef USE_FD
    kCAN_DataPhaseLstErrFlag   = CAN_PSR_DLEC_MASK, /*!< Last data phase error for FD/BRS frame */
    kCAN_LastESIFlag           = CAN_PSR_RESI_MASK, /*!< Last received message has ESI flag set */
    kCAN_LastBRSFlag           = CAN_PSR_RBRS_MASK, /*!< Last received message used baud rate switching */
    kCAN_FDReceivedFlag        = CAN_PSR_RFDF_MASK, /*!< An FD message has been received */
#endif
    kCAN_ProtocolExceptionFlag = CAN_PSR_PXE_MASK,  /*!< Protocol exception occurred */
    kCAN_TxDelayCompValueFlag  = CAN_PSR_TDCV_MASK  /*!< transmitter delay compensation value */
};

/*!
 * @brief CAN error status flags.
 *
 * The CAN Error Status enumerations is used to report current error of the CAN bus.
 * This enumerations should be used with kCAN_ErrorFlag in @ref _can_flags enumerations
 * to determine which error is generated.
 */
enum _can_error_flags
{
    kCAN_NoError       = 0, /*!< No errors */
    kCAN_StuffingError = 1, /*!< Stuffing error */
    kCAN_FormError     = 2, /*!< Form error */
    kCAN_AckError      = 3, /*!< No ACK received on transmission */
    kCAN_Bit1Error     = 4, /*!< Unable to send recessive bit */
    kCAN_Bit0Error     = 5, /*!< Unable to send dominant bit */
    kCAN_CrcError      = 6, /*!< Cyclic redundancy check error */
    kCAN_NoChange      = 7  /*!< No errors since last read of status */
};

/*!
 * @brief CAN global filter flags
 *
 * Defines what happens to messages that do not match any filter.
 * They can be placed into a FIFO or rejected
 */
enum _can_global_filter_flags
{
    kCAN_GlobalFilter_Standard_FIFO0  = (0UL << 4), /*!< Put non-matching 11-bit msgs into FIFO0 */
    kCAN_GlobalFilter_Standard_FIFO1  = (1UL << 4), /*!< Put non-matching 11-bit msgs into FIFO1 */
    kCAN_GlobalFilter_Standard_Reject = (2UL << 4), /*!< Reject non-matching 11-bit msgs */
    kCAN_GlobalFilter_Extended_FIFO0  = (0UL << 2), /*!< Put non-matching 29-bit msgs into FIFO0 */
    kCAN_GlobalFilter_Extended_FIFO1  = (1UL << 2), /*!< Put non-matching 29-bit msgs into FIFO1 */
    kCAN_GlobalFilter_Extended_Reject = (2UL << 2)  /*!< Reject non-matching 29-bit msgs */
};

#if defined(__CC_ARM)
#pragma anon_unions
#endif
/*! @brief CAN message frame structure. */
typedef struct _can_frame
{
    struct
    {
        uint32_t length      : 7; /*!< CAN frame payload length in bytes (Range: 0~64) */
        uint32_t type        : 1; /*!< CAN frame type (DATA or REMOTE) */
        uint32_t format      : 1; /*!< CAN frame identifier (STD or EXT format) */
        uint32_t proto       : 1; /*!< CAN frame protocol (CLASSIC or FD) */
        uint32_t bitratemode : 1; /*!< CAN frame bitrate mode (NOSWITCH or SWITCH) */
        uint32_t reserve1    : 5;
        uint16_t timestamp;       /*!< Receive timestamp */
    };
    struct
    {
        uint32_t id       : 29;  /*!< CAN frame identifier, should be set using CAN_ID_EXT() or CAN_ID_STD() macros */
        uint32_t reserve2 : 3;   /*!< Reserved for place holder */
    };
    union
    {
        struct
        {
            uint32_t dataWord[CAN_MAX_MESSAGE_WORDS]; /*!< CAN frame payload in words */
        };
        struct
        {
            uint8_t dataByte[CAN_MAX_MESSAGE_BYTES];  /*!< CAN frame payload in bytes */
        };
    };
} can_frame_t;

/*! @brief CAN Message Buffer transfer. */
typedef struct _can_mb_transfer
{
    can_frame_t *frame;     /*!< The buffer of CAN Message to be transfer. */
    uint8_t mbIdx;          /*!< The index of Message buffer used to transfer Message. */
} can_mb_transfer_t;

/*! @brief CAN Rx FIFO transfer. */
typedef struct _can_fifo_transfer
{
    can_frame_t *frame;     /*!< The buffer of CAN Message to be received from Rx FIFO. */
    uint8_t fifoIdx;        /*!< Number of the FIFO. */
} can_fifo_transfer_t;

/*! @brief CAN handle structure definition. */
typedef struct _can_handle can_handle_t;

/*! @brief CAN transfer callback function.
 *
 *  The CAN transfer callback returns a value from the underlying layer.
 *  If the status equals to kStatus_CAN_ErrorStatus, the result parameter is the Content of
 *  CAN status register which can be used to get the working status(or error status) of CAN module.
 *  If the status equals to other CAN Message Buffer transfer status, the result is the index of
 *  Message Buffer that generate transfer event.
 *  If the status equals to other CAN Message Buffer transfer status, the result is meaningless and should be
 *  Ignored.
 */
typedef void (*can_transfer_callback_t)(
        CAN_Type *base, can_handle_t *handle, status_t status, uint32_t result, void *userData);

/*! @brief CAN handle structure. */
typedef struct _can_handle
{
    can_transfer_callback_t callback;                             /*!< Callback function. */
    void *userData;                                               /*!< CAN callback function parameter.*/
    can_frame_t *volatile mbFrameBuf[CAN_NUM_RX_MESSAGE_BUFFERS]; /*!< The buffer for received data from Message Buffers. */
    can_frame_t *volatile rxFifoFrameBuf[CAN_NUM_RX_FIFOS];       /*!< The buffer for received data from Rx FIFO. */
    volatile uint8_t rxMbState[CAN_NUM_RX_MESSAGE_BUFFERS];       /*!< Rx Message Buffer transfer state. */
    volatile uint8_t txMbState[CAN_NUM_TX_MESSAGE_BUFFERS];       /*!< Tx Message Buffer transfer state. */
    volatile uint8_t rxFifoState[CAN_NUM_RX_FIFOS];               /*!< Rx FIFO transfer state. */
} can_handle_t;

/*! @brief CAN module configuration structure. */
typedef struct _can_config
{
    uint32_t nominalBaudRate;             /*!< CAN nominal baud rate in bps. */
#ifdef USE_FD
    uint32_t dataBaudRate;                /*!< CAN nominal baud rate in bps. */
#endif
    uint32_t baseAddress;                 /*!< RAM base address for buffers */
    uint32_t timestampClock_Hz;           /*!< Frequency of timestamp clock in Hz, must be at least sysclock / 2048, zero to disable */
    bool     rejectStandardRTR;           /*!< Set to TRUE to reject all 11-bit RTRs */
    bool     rejectExtendedRTR;           /*!< Set to TRUE to reject all 29-bit RTRs */
    bool     enableLoopBack;              /*!< Set to TRUE to enable loopback mode. */
#ifdef USE_FD
    bool     enableNonISOMode;            /*!< Set to TRUE for CAN FD non-ISO mode. Only use for communication with 'old' FD controllers. */
    bool     disableFD;                   /*!< Set to TRUE to disable FD mode. */
#endif
} can_config_t;

/*! @brief CAN protocol timing characteristic configuration structure. */
typedef struct _can_timing_config
{
    uint8_t  preDivider;        /*!< Global Clock Division Factor. */
    uint16_t nominalPrescaler;  /*!< Nominal clock prescaler. */
    uint8_t  nominalRJumpwidth; /*!< Nominal Re-sync Jump Width. */
    uint8_t  nominalPhaseSeg1;  /*!< Nominal Phase Segment 1. */
    uint8_t  nominalPhaseSeg2;  /*!< Nominal Phase Segment 2. */
    uint8_t  nominalPropSeg;    /*!< Nominal Propagation Segment. */
#ifdef USE_FD
    uint8_t  dataPrescaler;     /*!< Data clock prescaler. */
    uint8_t  dataRJumpwidth;    /*!< Data Re-sync Jump Width. */
    uint8_t  dataPhaseSeg1;     /*!< Data Phase Segment 1. */
    uint8_t  dataPhaseSeg2;     /*!< Data Phase Segment 2. */
    uint8_t  dataPropSeg;       /*!< Data Propagation Segment. */
#endif
} can_timing_config_t;

/*! @brief CAN Rx FIFO configure structure. */
typedef struct _can_rx_fifo_config
{
    uint8_t idFilterNum; /*!< The quantity of filter elements. */
} can_rx_fifo_config_t;

/*! @brief CAN interrupt configuration structure, default settings all disabled.
 *
 * This structure contains the settings for all of the CAN Module interrupt configurations.
 */
enum _can_interrupt_enable
{
    kCAN_BusOffInterruptEnable        = CAN_IE_BOE_MASK,   /*!< Bus Off interrupt. */
    kCAN_ErrorInterruptEnable         = CAN_IE_EPE_MASK,   /*!< Error interrupt. */
    kCAN_WarningInterruptEnable       = CAN_IE_EWE_MASK,   /*!< Warning interrupt. */
    kCAN_RxBufferInterruptEnable      = CAN_IE_DRXE_MASK,  /*!< Dedicated rx buffer receive interrupt. */
    kCAN_TxCompletedInterruptEnable   = CAN_IE_TCE_MASK,   /*!< Tx completed interrupt. */
    kCAN_RxFIFO1InterruptEnable       = CAN_IE_RF1NE_MASK, /*!< Rx FIFO1 receive interrupt. */
    kCAN_RxFIFO0InterruptEnable       = CAN_IE_RF0NE_MASK, /*!< Rx FIFO0 receive interrupt. */
    kCAN_TimestampWrapInterruptEnable = CAN_IE_TSWE_MASK   /*!< Timestamp wraparound interrupt. */
};

/*!
 * @brief Enables CAN interrupts according to provided mask.
 *
 * This function enables the CAN interrupts according to provided mask. The mask
 * is a logical OR of enumeration members, see @ref _can_interrupt_enable.
 *
 * @param base CAN peripheral base address.
 * @param mask The interrupts to enable. Logical OR of @ref _can_interrupt_enable.
 */
__STATIC_INLINE void CAN_EnableInterrupts(CAN_Type *base, uint32_t mask)
{
    base->IE |= mask;
}

/*!
 * @brief Disables CAN interrupts according to provided mask.
 *
 * This function disables the CAN interrupts according to provided mask. The mask
 * is a logical OR of enumeration members, see @ref _can_interrupt_enable.
 *
 * @param base CAN peripheral base address.
 * @param mask The interrupts to disable. Logical OR of @ref _can_interrupt_enable.
 */
__STATIC_INLINE void CAN_DisableInterrupts(CAN_Type *base, uint32_t mask)
{
    base->IE &= ~(mask);
}

/*!
 * @brief Enables CAN Message Buffer interrupts.
 *
 * This function enables the interrupts of given Message Buffers
 *
 * @param base CAN peripheral base address.
 * @param mask The ORed CAN Message Buffer mask.
 */
__STATIC_INLINE void CAN_EnableMbInterrupts(CAN_Type *base, uint32_t mask)
{
    base->TXBTIE |= mask;
}

/*!
 * @brief Disables CAN Message Buffer interrupts.
 *
 * This function disables the interrupts of given Message Buffers
 *
 * @param base CAN peripheral base address.
 * @param mask The ORed CAN Message Buffer mask.
 */
__STATIC_INLINE void CAN_DisableMbInterrupts(CAN_Type *base, uint32_t mask)
{
    base->TXBTIE &= ~mask;
}

/*!
 * @brief Gets the CAN module interrupt flags.
 *
 * This function gets all CAN status flags. The flags are returned as the logical
 * OR value of the enumerators @ref _can_flags. To check the specific status,
 * compare the return value with enumerators in @ref _can_flags.
 *
 * @param base CAN peripheral base address.
 * @return CAN status flags which are ORed by the enumerators in the _can_flags.
 */
__STATIC_INLINE uint32_t CAN_GetStatusFlags(CAN_Type *base)
{
    return base->PSR;
}

/*!
 * @brief Clears status flags with the provided mask.
 *
 * This function clears the CAN status flags with a provided mask. An automatically cleared flag
 * can't be cleared by this function.
 *
 * @param base CAN peripheral base address.
 * @param mask The status flags to be cleared, it is logical OR value of @ref _can_flags.
 */
__STATIC_INLINE void CAN_ClearStatusFlags(CAN_Type *base, uint32_t mask)
{
    /* not used in this module as all status register flags are read only */
}

/*!
 * @brief Gets the CAN Bus Error Counter value.
 *
 * This function gets the CAN Bus Error Counter value for both Tx and
 * Rx direction. These values may be needed in the upper layer error handling.
 *
 * @param base CAN peripheral base address.
 * @param txErrBuf Buffer to store Tx Error Counter value.
 * @param rxErrBuf Buffer to store Rx Error Counter value.
 */
__STATIC_INLINE void CAN_GetBusErrCount(CAN_Type *base, uint8_t *txErrBuf, uint8_t *rxErrBuf)
{
    if (txErrBuf)
    {
        *txErrBuf = (uint8_t)((base->ECR >> CAN_ECR_TEC_SHIFT) & CAN_ECR_TEC_MASK);
    }

    if (rxErrBuf)
    {
        *rxErrBuf = (uint8_t)((base->ECR >> CAN_ECR_REC_SHIFT) & CAN_ECR_REC_MASK);
    }
}

/*!
 * @brief Sets the CAN protocol timing characteristic.
 *
 * This function gives user settings to CAN bus timing characteristic.
 * The function is for an experienced user. For less experienced users, call
 * the CAN_Init() and fill the baud rate field with a desired value.
 * This provides the default timing characteristics to the module.
 *
 * Note that calling CAN_SetTimingConfig() overrides the baud rate set
 * in CAN_Init().
 *
 * @param base CAN peripheral base address.
 * @param config Pointer to the timing configuration structure.
 */
void CAN_SetTimingConfig(CAN_Type *base, const can_timing_config_t *config);

/*!
 * @brief Enables or disables the CAN module operation.
 *
 * This function enables or disables the CAN module.
 *
 * @param base CAN base pointer.
 * @param enable true to enable, false to disable.
 */
void CAN_Enable(CAN_Type *base, bool enable);

/*!
 * @brief Initializes a CAN instance.
 *
 * This function initializes the CAN module with user-defined settings.
 *
 * @param base CAN peripheral base address.
 * @param config Pointer to user-defined configuration structure.
 * @param sourceClock_Hz CAN Protocol Engine clock source frequency in Hz.
 */
void CAN_Init(CAN_Type *base, const can_config_t *config, uint32_t sourceClock_Hz);

/*!
 * @brief Configures the CAN Rx FIFO.
 *
 * This function configures the Rx FIFO with given Rx FIFO configuration.
 *
 * @param base CAN peripheral base address.
 * @param fifoIdx Number of FIFO, 0 or 1
 * @param config Pointer to CAN Rx FIFO configuration structure.
 * @param enable Enable/Disable Rx FIFO.
 *               - true: Enable Rx FIFO.
 *               - false: Disable Rx FIFO.
 */
void CAN_SetRxFifoConfig(CAN_Type *base, uint8_t fifoIdx, const can_rx_fifo_config_t *config, bool enable);

/*!
 * @brief De-initializes a CAN instance.
 *
 * This function disable the CAN module clock and set all register value
 * to reset value.
 *
 * @param base CAN peripheral base address.
 */
void CAN_Deinit(CAN_Type *base);

/*!
 * @brief Aborts transmission on a CAN transmit message buffer.
 *
 * This function aborts the previous transmission and cleans the Message Buffer
 *
 * @param base CAN peripheral base address.
 * @param mbIdx The Message Buffer index.
 */
void CAN_AbortTxMb(CAN_Type *base, uint8_t mbIdx);

/*!
 * @brief Writes a CAN Message to Transmit Message Buffer.
 *
 * This function writes a CAN Message to the specified Transmit Message Buffer
 * and changes the Message Buffer state to start CAN Message transmit. After
 * that the function returns immediately.
 *
 * @param base CAN peripheral base address.
 * @param mbIdx The CAN Message Buffer index.
 * @param txFrame Pointer to CAN message frame to be sent.
 * @retval kStatus_Success - Write Tx Message Buffer Successfully.
 * @retval kStatus_Fail    - Tx Message Buffer is currently in use.
 */
status_t CAN_WriteTxMb(CAN_Type *base, uint8_t mbIdx, const can_frame_t *txFrame);

/*!
 * @brief Performs a polling send transaction on the CAN bus.
 *
 * Note that a transfer handle does not need to be created  before calling this API.
 *
 * @param base CAN peripheral base pointer.
 * @param mbIdx The CAN Message Buffer index.
 * @param txFrame Pointer to CAN message frame to be sent.
 * @retval kStatus_Success - Write Tx Message Buffer Successfully.
 * @retval kStatus_Fail    - Tx Message Buffer is currently in use.
 */
status_t CAN_TransferSendBlocking(CAN_Type *base, uint8_t mbIdx, can_frame_t *txFrame);

/*!
 * @brief Sets the CAN receive global mask.
 *
 * This function sets the global mask for CAN in a matching process.
 *
 * @param base CAN peripheral base address.
 * @param filter Rx Global Mask value.
 */
void CAN_SetRxGlobalMask(CAN_Type *base, uint32_t filter);

/*!
 * @brief Sets the CAN receive individual mask.
 *
 * This function sets the individual mask for CAN matching process.
 *
 * @param base CAN peripheral base address.
 * @param maskIdx The Index of individual Mask.
 * @param filter Rx Individual filter value.
 */
void CAN_SetRxIndividualMask(CAN_Type *base, uint8_t maskIdx, uint32_t filter);

/*!
 * @brief Sets the CAN receive extended individual mask.
 *
 * This function sets the extended individual mask for CAN matching process.
 *
 * @param base CAN peripheral base address.
 * @param maskIdx The Index of individual Mask.
 * @param filterLow Rx Individual filter low value.
 * @param filterHigh Rx Individual filter high value.
 */
void CAN_SetRxExtIndividualMask(CAN_Type *base, uint8_t maskIdx, uint32_t filterLow, uint32_t filterHigh);

/*!
 * @brief Reads a CAN Message from Rx FIFO.
 *
 * This function reads a CAN message from the CAN build-in Rx FIFO.
 *
 * @param base CAN peripheral base address.
 * @param fifoIdx Number of the FIFO, 0 or 1
 * @param rxFrame Pointer to CAN message frame structure for reception.
 * @retval kStatus_Success - Read Message from Rx FIFO successfully.
 * @retval kStatus_CAN_RxOverflow - Rx FIFO is already overflowed and has been read successfully.
 * @retval kStatus_Fail    - Rx FIFO is not enabled.
 */
status_t CAN_ReadRxFifo(CAN_Type *base, uint8_t fifoIdx, can_frame_t *rxFrame);

/*!
 * @brief Performs a polling receive transaction from Rx FIFO on the CAN bus.
 *
 * Note that a transfer handle does not need to be created  before calling this API.
 *
 * @param base CAN peripheral base pointer.
 * @param fifoIdx Number of the FIFO, 0 or 1
 * @param rxFrame Pointer to CAN message frame structure for reception.
 * @retval kStatus_Success - Read Message from Rx FIFO successfully.
 * @retval kStatus_CAN_RxOverflow - Rx FIFO is already overflowed and has been read successfully.
 * @retval kStatus_Fail    - Rx FIFO is not enabled.
 */
status_t CAN_TransferReceiveFifoBlocking(CAN_Type *base, uint8_t fifoIdx, can_frame_t *rxFrame);

/*!
 * @brief Reads a CAN Message from Receive Message Buffer.
 *
 * This function reads a CAN message from a specified Receive Message Buffer.
 * The function fills a receive CAN message frame structure with
 * just received data and activates the Message Buffer again.
 * The function returns immediately.
 *
 * @param base CAN peripheral base address.
 * @param mbIdx The CAN Message Buffer index.
 * @param rxFrame Pointer to CAN message frame structure for reception.
 * @retval kStatus_Success            - Rx Message Buffer is full and has been read successfully.
 * @retval kStatus_Fail               - Rx Message Buffer is empty.
 */
status_t CAN_ReadRxMb(CAN_Type *base, uint8_t mbIdx, can_frame_t *rxFrame);

/*!
 * @brief Performs a polling receive transaction on the CAN bus.
 *
 * Note that a transfer handle does not need to be created  before calling this API.
 *
 * @param base CAN peripheral base pointer.
 * @param mbIdx The CAN Message Buffer index.
 * @param rxFrame Pointer to CAN message frame structure for reception.
 * @retval kStatus_Success            - Rx Message Buffer is full and has been read successfully.
 * @retval kStatus_Fail               - Rx Message Buffer is empty.
 */
status_t CAN_TransferReceiveBlocking(CAN_Type *base, uint8_t mbIdx, can_frame_t *rxFrame);

/*!
 * @brief Aborts the interrupt driven message receive process.
 *
 * This function aborts the interrupt driven message receive process.
 *
 * @param base CAN peripheral base address.
 * @param handle CAN handle pointer.
 * @param mbIdx The CAN Message Buffer index.
 */
void CAN_TransferAbortReceive(CAN_Type *base, can_handle_t *handle, uint8_t mbIdx);

/*!
 * @brief Aborts the interrupt driven message receive from Rx FIFO process.
 *
 * This function aborts the interrupt driven message receive from Rx FIFO process.
 *
 * @param base CAN peripheral base address.
 * @param fifoIdx Number of FIFO, 0 or 1
 * @param handle CAN handle pointer.
 */
void CAN_TransferAbortReceiveFifo(CAN_Type *base, uint8_t fifoIdx, can_handle_t *handle);

/*!
 * @brief Aborts the interrupt driven message send process.
 *
 * This function aborts the interrupt driven message send process.
 *
 * @param base CAN peripheral base address.
 * @param handle CAN handle pointer.
 * @param mbIdx The CAN Message Buffer index.
 */
void CAN_TransferAbortSend(CAN_Type *base, can_handle_t *handle, uint8_t mbIdx);

/*!
 * @brief CAN IRQ handle function.
 *
 * This function handles the CAN Error, the Message Buffer, and the Rx FIFO IRQ request.
 *
 * @param base CAN peripheral base address.
 * @param handle CAN handle pointer.
 */
void CAN_TransferHandleIRQ(CAN_Type *base, can_handle_t *handle);

/*!
 * @brief Sends a message using IRQ.
 *
 * This function sends a message using IRQ. This is a non-blocking function, which returns
 * right away. When messages have been sent out, the send callback function is called.
 *
 * @param base CAN peripheral base address.
 * @param handle CAN handle pointer.
 * @param xfer CAN Message Buffer transfer structure. See the #can_mb_transfer_t.
 * @retval kStatus_Success        Start Tx Message Buffer sending process successfully.
 * @retval kStatus_Fail           Write Tx Message Buffer failed.
 * @retval kStatus_CAN_TxBusy Tx Message Buffer is in use.
 */
status_t CAN_TransferSendNonBlocking(CAN_Type *base, can_handle_t *handle, can_mb_transfer_t *xfer);

/*!
 * @brief Receives a message using IRQ.
 *
 * This function receives a message using IRQ. This is non-blocking function, which returns
 * right away. When the message has been received, the receive callback function is called.
 *
 * @param base CAN peripheral base address.
 * @param handle CAN handle pointer.
 * @param xfer CAN Message Buffer transfer structure. See the #can_mb_transfer_t.
 * @retval kStatus_Success        - Start Rx Message Buffer receiving process successfully.
 * @retval kStatus_CAN_RxBusy - Rx Message Buffer is in use.
 */
status_t CAN_TransferReceiveNonBlocking(CAN_Type *base, can_handle_t *handle, can_mb_transfer_t *xfer);

/*!
 * @brief Receives a message from Rx FIFO using IRQ.
 *
 * This function receives a message using IRQ. This is a non-blocking function, which returns
 * right away. When all messages have been received, the receive callback function is called.
 *
 * @param base CAN peripheral base address.
 * @param handle CAN handle pointer.
 * @param xfer CAN Rx FIFO transfer structure. See the @ref can_fifo_transfer_t.
 * @retval kStatus_Success            - Start Rx FIFO receiving process successfully.
 * @retval kStatus_CAN_RxFifoBusy - Rx FIFO is currently in use.
 * @retval kStatus_Fail               - Configuration of FIFO has failed.
 */
status_t CAN_TransferReceiveFifoNonBlocking(CAN_Type *base,
                                                can_handle_t *handle,
                                                can_fifo_transfer_t *xfer);

/*!
 * @brief Initializes the CAN handle.
 *
 * This function initializes the CAN handle which can be used for other CAN
 * transactional APIs. Usually, for a specified CAN instance,
 * call this API once to get the initialized handle.
 *
 * @param base CAN peripheral base address.
 * @param handle CAN handle pointer.
 * @param callback The callback function.
 * @param userData The parameter of the callback function.
 */
void CAN_TransferCreateHandle(CAN_Type *base,
                                  can_handle_t *handle,
                                  can_transfer_callback_t callback,
                                  void *userData);

/*!
 * @brief Get the default configuration structure.
 *
 * This function initializes the CAN configure structure to default value. The default
 * value are:
 *  config->nominalBaudRate   = 125000;
 *  config->dataBaudRate      = 0;
 *  config->baseAddress       = 0x20010000;
 *  config->timestampClock_Hz = 0;
 *  config->rejectStandardRTR = TRUE;
 *  config->rejectExtendedRTR = TRUE;
 *  config->enableLoopBack    = FALSE;
 *  config->enableNonISOMode  = FALSE;
 *  config->disableFD         = FALSE;
 *
 * @param config Pointer to CAN configuration structure.
 */
void CAN_GetDefaultConfig(can_config_t *config);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_5461X_H_ */
