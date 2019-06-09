/*
 * @brief LPC5461X CAN-FD driver
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2017
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * NXP products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licenser disclaim any and
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

#include "can.h"
#include "fsl_device_registers.h"
#include <string.h>

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define CAN_TSCC_TSS_DISABLED   (0x00 << 0)
#define CAN_TSCC_TSS_INTCOUNTER (0x01 << 0)
#define CAN_TSCC_TSS_EXTCOUNTER (0x02 << 0)

#define CAN_TCTRL_ETCE      (1UL << 31)

#define CAN_RXFS_RFL CAN_RXF0S_RF0L_MASK

/** Access to tx and rx message buffer fields - ID */
#define CAN_BUF_ID_XTD_BIT    30
#define CAN_BUF_ID_RTR_BIT    29
#define CAN_BUF_ID_ESI        (1UL << 31)
#define CAN_BUF_ID_XTD        (1UL << CAN_BUF_ID_XTD_BIT)
#define CAN_BUF_ID_RTR        (1UL << CAN_BUF_ID_RTR_BIT)
#define CAN_BUF_ID_STDID_MASK (0x7FFUL << 18)
#define CAN_BUF_ID_EXTID_MASK (0x1FFFFFFFUL << 0)

/** Acces to tx and rx message buffer fields - config */
#define CAN_BUF_CONFIG_ANMF      (1UL << 31)
#define CAN_BUF_CONFIG_FIDX_MASK (0x7FUL << 24)
#ifdef USE_FD
#define CAN_BUF_CONFIG_FDF       (1UL << 21)
#define CAN_BUF_CONFIG_BRS       (1UL << 20)
#endif
#define CAN_BUF_CONFIG_DLC_MASK  (0xFUL << 16)
#define CAN_BUF_CONFIG_RXTS_MASK (0xFFFFUL << 0)

/*! @brief Maximum nominal baudrate in bps. This is a CAN limit and can't be changed. */
#define MAX_NOMINAL_BAUDRATE 1000000
/*! @brief Minimum number of time quanta in a bit.
    this value ensures that at 87.5% SP TSEG2 is at least two
    which is the minimum for M_CAN */
#define MIN_TIME_QUANTA 9
/*! @brief Maximum number of time quanta in a bit. */
#define MAX_TIME_QUANTA 20
/*! @brief Maximum receive FIFO size in messages. */
#define MAX_RX_FIFO_SIZE 64

/*! @brief Number of CAN controllers on-chip. */
#define FSL_FEATURE_SOC_CAN_COUNT 2

/*! @brief CAN Internal State. */
enum _can_state
{
    kCAN_StateIdle     = 0, /*!< MB/RxFIFO idle.*/
    kCAN_StateRxData   = 1, /*!< MB receiving.*/
    kCAN_StateRxRemote = 2, /*!< MB receiving remote reply.*/
    kCAN_StateTxData   = 3, /*!< MB transmitting.*/
    kCAN_StateTxRemote = 4, /*!< MB transmitting remote request.*/
    kCAN_StateRxFifo   = 5, /*!< RxFIFO receiving.*/
};

#if defined(__CC_ARM)
#pragma anon_unions
#endif
/*! @brief layout of transmit and receive message buffers. */
typedef struct {
    uint32_t ID;     /*!< Message identifier and associated flags. */
    uint32_t CONFIG; /*!< Buffer configuration. */
    union {
        uint32_t DATA32[CAN_MAX_MESSAGE_WORDS]; /*!< Word access to buffer data. */
        uint8_t DATA8[CAN_MAX_MESSAGE_BYTES];   /*!< Byte access to buffer data. */
    };
} CAN_BUF_T;

#if defined(__CC_ARM)
#pragma anon_unions
#endif
/*! @brief Layout of standard message filter. */
typedef struct {
  union
  {
      struct
      {
          uint32_t SFID2     : 11;
          uint32_t reserved1 : 5;
          uint32_t SFID1     : 11;
          uint32_t SFEC      : 3;
          uint32_t SFT       : 2;
      };
      struct
      {
          uint32_t VALUE; /*!< Access to filter as a word. */
      };
  };
} CAN_STDFILTER_T;

#if defined(__CC_ARM)
#pragma anon_unions
#endif
/*! @brief Layout of extended message filter. */
typedef struct {
  union
  {
      struct
      {
          uint32_t EFID1     : 29;
          uint32_t EFEC      : 3;
          uint32_t EFID2     : 29;
          uint32_t reserved1 : 1;
          uint32_t EFT       : 2;
      };
      struct
      {
          uint32_t LOWVALUE;  /*!< Access to filter low word. */
          uint32_t HIGHVALUE; /*!< Access to filter high word. */
      };
  };
} CAN_EXTFILTER_T;

/*! @brief Array of can handles, one handle per CAN controller. */
static can_handle_t *s_canHandle[FSL_FEATURE_SOC_CAN_COUNT];

/*! @brief Array of can peripheral base addresses. */
static CAN_Type *const s_canBases[] = CAN_BASE_PTRS;

/*! @brief Next free location in RAM for CAN data storage. */
static uint16_t s_ramOffset;
/*! @brief Keeps track of the size of each receive FIFO. */
static uint8_t s_rxFIFOSize[CAN_NUM_RX_FIFOS];

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*!
 * @brief Converts number of bytes in a message into a
 * Data Length Code
 *
 * @param numberOfBytes Number of bytes in a message
 * @return Data Length Code
 */
static uint8_t CAN_EncodeDLC(uint8_t numberOfBytes)
{
    if (numberOfBytes <= 8) return numberOfBytes;
    else if (numberOfBytes <= 12) return 9;
    else if (numberOfBytes <= 16) return 10;
    else if (numberOfBytes <= 20) return 11;
    else if (numberOfBytes <= 24) return 12;
    else if (numberOfBytes <= 32) return 13;
    else if (numberOfBytes <= 48) return 14;
    else return 15;
}

/*!
 * @brief Converts a Data Length Code into a number
 * of message bytes
 *
 * @param dlc Data Length Code
 * @return Number of bytes in a message
 */
static uint8_t CAN_DecodeDLC(uint8_t dlc)
{
    if (dlc <= 8) return dlc;
    else if (dlc == 9) return 12;
    else if (dlc == 10) return 16;
    else if (dlc == 11) return 20;
    else if (dlc == 12) return 24;
    else if (dlc == 13) return 32;
    else if (dlc == 14) return 48;
    else return 64;
}

/*!
 * @brief Converts number of bytes in a message into a
 * a FIFO/buffer size encoding
 *
 * @param numberOfBytes Number of bytes in a message
 * @return FIFO/buffer size encoding
 */
static uint8_t CAN_NumberofBytestoFieldSize(uint8_t numberOfBytes)
{
    if (numberOfBytes <= 8) return 0;
    else if (numberOfBytes <= 12) return 1;
    else if (numberOfBytes <= 16) return 2;
    else if (numberOfBytes <= 20) return 3;
    else if (numberOfBytes <= 24) return 4;
    else if (numberOfBytes <= 32) return 5;
    else if (numberOfBytes <= 48) return 6;
    else return 7;
}

/*!
 * @brief Get the CAN instance from peripheral base address.
 *
 * @param base CAN peripheral base address.
 * @return CAN instance.
 */
static uint32_t CAN_GetInstance(CAN_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < FSL_FEATURE_SOC_CAN_COUNT; instance++)
    {
        if (s_canBases[instance] == base)
        {
            break;
        }
    }

    return instance;
}

/*!
 * @brief Calculates the segment values for a single bit time for nominal and data baudrates
 *
 * @param nominalBaudRate The nominal speed in bps
 * @param dataBaudRate The data speed in bps
 * @param ntq Number of nominal time quanta per bit
 * @param dtq Number of data time quanta per bit
 * @param pconfig Passed is a configuration structure, on return the configuration is stored
 *                in the structure
 */
static void CAN_GetSegments(uint32_t nominalBaudRate, uint32_t dataBaudRate, uint32_t ntq,
        uint32_t dtq, can_timing_config_t *pconfig)
{
    float ideal_sp;
    int p1;

    /* get ideal sample point */
    if (nominalBaudRate >= 1000000)     ideal_sp = 0.750;
    else if (nominalBaudRate >= 800000) ideal_sp = 0.800;
    else                                ideal_sp = 0.875;

    /* distribute time quanta */
    p1 = (int)(ntq * ideal_sp);
    /* can controller doesn't separate prop seg and phase seg 1 */
    pconfig->nominalPropSeg = 0;
    /* subtract one TQ for sync seg */
    pconfig->nominalPhaseSeg1 = p1 - 1;
    pconfig->nominalPhaseSeg2 = ntq - p1;
    /* sjw is 20% of total TQ, rounded to nearest int */
    pconfig->nominalRJumpwidth = (ntq + (5 - 1)) / 5;

#ifdef USE_FD
    /* if using baud rate switching then distribute time quanta for data rate */
    if (dtq > 0)
    {
        /* get ideal sample point */
        if (dataBaudRate >= 1000000)     ideal_sp = 0.750;
        else if (dataBaudRate >= 800000) ideal_sp = 0.800;
        else                             ideal_sp = 0.875;

        /* distribute time quanta */
        p1 = (int)(dtq * ideal_sp);
        /* can controller doesn't separate prop seg and phase seg 1 */
        pconfig->dataPropSeg = 0;
        /* subtract one TQ for sync seg */
        pconfig->dataPhaseSeg1 = p1 - 1;
        pconfig->dataPhaseSeg2 = dtq - p1;
        /* sjw is 20% of total TQ, rounded to nearest int */
        pconfig->dataRJumpwidth = (dtq + (5 - 1)) / 5;
    }
    else
    {
        pconfig->dataPropSeg = 0;
        pconfig->dataPhaseSeg1 = 0;
        pconfig->dataPhaseSeg2 = 0;
        pconfig->dataRJumpwidth = 0;
    }
#endif
}

/*!
 * @brief Calculates the CAN controller timing values for specific baudrates
 *
 * @param nominalBaudRate The nominal speed in bps
 * @param dataBaudRate The data speed in bps. Zero to disable baudrate switching
 * @param pconfig Passed is a configuration structure, on return the configuration is stored
 *                in the structure
 * @return true if timing configuration found, false if failed to find configuration
 */
static int CAN_CalculateTimingValues(uint32_t nominalBaudRate,
#ifdef USE_FD
        uint32_t dataBaudRate,
#endif
        uint32_t sourceClock_Hz, can_timing_config_t *pconfig)
{
    int nclk;
    int nclk2;
    int ntq;
#ifdef USE_FD
    int dclk;
    int dclk2;
    int dtq;
#endif

    /* observe baud rate maximums */
    if (nominalBaudRate > MAX_NOMINAL_BAUDRATE) nominalBaudRate = MAX_NOMINAL_BAUDRATE;

    for (ntq = MAX_TIME_QUANTA; ntq >= MIN_TIME_QUANTA; ntq--)
    {
        nclk = nominalBaudRate * ntq;

        for (pconfig->nominalPrescaler = 0x001; pconfig->nominalPrescaler <= 0x400; (pconfig->nominalPrescaler)++)
        {
            nclk2 = nclk * pconfig->nominalPrescaler;

            if (((sourceClock_Hz / nclk2) <= 256) && ((float)(sourceClock_Hz) / nclk2) == (sourceClock_Hz / nclk2))
            {
                pconfig->preDivider = sourceClock_Hz / nclk2;
									//pconfig->preDivider = 1;
#ifdef USE_FD
                /* if not using baudrate switch then we are done */
                if (!dataBaudRate)
                {
                    dtq = 0;
                    pconfig->dataPrescaler = 0;
                    CAN_GetSegments(nominalBaudRate, dataBaudRate, ntq, dtq, pconfig);
                    return true;
                }

                /* if baudrates are the same and the solution for nominal will work for
                data, then use the nominal settings for both */
                if ((dataBaudRate == nominalBaudRate) && pconfig->nominalPrescaler <= 0x20)
                {
                    dtq = ntq;
                    pconfig->dataPrescaler = (uint8_t)pconfig->nominalPrescaler;
                    CAN_GetSegments(nominalBaudRate, dataBaudRate, ntq, dtq, pconfig);
                    return true;
                }

                /* calculate data settings */
                for (dtq = MAX_TIME_QUANTA; dtq >= MIN_TIME_QUANTA; dtq--)
                {
                    dclk = dataBaudRate * dtq;

                    for (pconfig->dataPrescaler = 0x01; pconfig->dataPrescaler <= 0x20; (pconfig->dataPrescaler)++)
                    {
                        dclk2 = dclk * pconfig->dataPrescaler;

                        if ((float)(sourceClock_Hz) / dclk2 == pconfig->preDivider)
                        {
                            CAN_GetSegments(nominalBaudRate, dataBaudRate, ntq, dtq, pconfig);
                            return true;
                        }
                    }
                }
#else
                CAN_GetSegments(nominalBaudRate, 0, ntq, 0, pconfig);
                return true;                
#endif
            }
        }
    }

    /* failed to find solution */
    return false;
}

/*!
 * @brief Reads a message frame from a receive buffer
 * @param rxBuffer Buffer to read from
 * @param Location to store read message
 */
static void CAN_ReadRxMessage(CAN_BUF_T *rxBuffer, can_frame_t *rxFrame)
{
    int b;

    /* if 29-bit ID */
    if (rxBuffer->ID & CAN_BUF_ID_XTD)
    {
        rxFrame->id = rxBuffer->ID & 0x1FFFFFFF;
        rxFrame->format = kCAN_FrameFormatExtend;
    }
    /* if 11-bit ID */
    else
    {
        rxFrame->id = (rxBuffer->ID >> 18) & 0x7FF;
        rxFrame->format = kCAN_FrameFormatStandard;
    }

#ifdef USE_FD
    if (rxBuffer->CONFIG & CAN_BUF_CONFIG_FDF)
        rxFrame->proto = kCAN_ProtoTypeFD;
    else
#endif
        rxFrame->proto = kCAN_ProtoTypeClassic;

#ifdef USE_FD
    if (rxBuffer->CONFIG & CAN_BUF_CONFIG_BRS)
        rxFrame->bitratemode = kCAN_BitrateModeTypeSwitch;
    else
        rxFrame->bitratemode = kCAN_BitrateModeTypeNoSwitch;
#endif
    
    rxFrame->timestamp = rxBuffer->CONFIG & CAN_BUF_CONFIG_RXTS_MASK;

    if (rxBuffer->ID & CAN_BUF_ID_RTR)
        rxFrame->type = kCAN_FrameTypeRemote;
    else
        rxFrame->type = kCAN_FrameTypeData;

    rxFrame->length = CAN_DecodeDLC((rxBuffer->CONFIG >> 16) & 0xF);

    for (b = 0; b < (int)(rxFrame->length + (4 - 1)) / 4; b++)
    {
        rxFrame->dataWord[b] = rxBuffer->DATA32[b];
    }
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

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
void CAN_SetTimingConfig(CAN_Type *base, const can_timing_config_t *config)
{
#ifdef USE_FD
    uint32_t *pDBTP;
#endif
  
    /* configuration change enable */
    base->CCCR |= CAN_CCCR_CCE_MASK;

    switch (CAN_GetInstance(base))
    {
        case 0:
            SYSCON->CAN0CLKDIV = config->preDivider - 1;
            break;

        case 1:
            SYSCON->CAN1CLKDIV = config->preDivider - 1;
            break;
    }

    /* nominal bit rate */
    base->NBTP = (((config->nominalRJumpwidth & 0x7F) - 1) << 25) +
                 (((config->nominalPrescaler & 0x1FF) - 1) << 16) +
                 ((((config->nominalPhaseSeg1 + config->nominalPropSeg) & 0xFF) - 1) << 8) +
                 (((config->nominalPhaseSeg2 & 0x7F) - 1) << 0);

#ifdef USE_FD
    /* data bit rate */
    /* base->DBTP */
    pDBTP = (((uint32_t *)base) + 0x03);
    *pDBTP = (((config->dataPrescaler & 0x1F) - 1) << 16) +
             ((((config->dataPhaseSeg1 + config->dataPropSeg) & 0x1F) - 1) << 8) +
             (((config->dataPhaseSeg2 & 0xF) - 1) << 4) +
             (((config->dataRJumpwidth & 0xF) - 1) << 0);
#endif
}

/*!
 * @brief Enables or disables the CAN module operation.
 *
 * This function enables or disables the CAN module.
 *
 * @param base CAN base pointer.
 * @param enable true to enable, false to disable.
 */
void CAN_Enable(CAN_Type *base, bool enable)
{
    if (enable)
    {
        /* start operation */
        base->CCCR &= ~(CAN_CCCR_CCE_MASK | CAN_CCCR_INIT_MASK);
        while (base->CCCR & CAN_CCCR_INIT_MASK);
    }
    else
    {
        /* init mode */
        base->CCCR |= CAN_CCCR_INIT_MASK;
        while (!(base->CCCR & CAN_CCCR_INIT_MASK));
    }
}

/*!
 * @brief Initializes a CAN instance.
 *
 * This function initializes the CAN module with user-defined settings.
 *
 * @param base CAN peripheral base address.
 * @param config Pointer to user-defined configuration structure.
 * @param sourceClock_Hz CAN Protocol Engine clock source frequency in Hz.
 */
void CAN_Init(CAN_Type *base, const can_config_t *config, uint32_t sourceClock_Hz)
{
    can_timing_config_t timingConfig;
    int f;
    int timestampdivisor;

    /* reset offset */
    s_ramOffset = 0;

    switch (CAN_GetInstance(base))
    {
        case 0:
            CLOCK_EnableClock(kCLOCK_Mcan0);
            RESET_PeripheralReset(kMCAN0_RST_SHIFT_RSTn);
            NVIC_EnableIRQ(CAN0_IRQ0_IRQn);
            NVIC_EnableIRQ(CAN0_IRQ1_IRQn);
            break;

        case 1:
            CLOCK_EnableClock(kCLOCK_Mcan1);
            RESET_PeripheralReset(kMCAN1_RST_SHIFT_RSTn);
            NVIC_EnableIRQ(CAN1_IRQ0_IRQn);
            NVIC_EnableIRQ(CAN1_IRQ1_IRQn);
            break;
    }

    /* configuration change enable */
    base->CCCR |= CAN_CCCR_CCE_MASK;
#ifdef USE_FD
    /* enable FD and baud-rate switching */
    base->CCCR |= CAN_CCCR_BRSE_MASK;

    if (!config->disableFD)
    {
        base->CCCR |= CAN_CCCR_FDOE_MASK;
    }
#endif
    
    /* no rx FIFOs */
    for (f = 0; f < CAN_NUM_RX_FIFOS; f++) s_rxFIFOSize[f] = 0;
    base->RXF0C = 0;
    base->RXF1C = 0;

    /* calculate and apply timing */
    if (CAN_CalculateTimingValues(config->nominalBaudRate,
#ifdef USE_FD
      config->dataBaudRate,
#endif
      sourceClock_Hz, &timingConfig))
    {
      //CAN_SetTimingConfig(base, &timingConfig);
    }

	

		/*!< Global Clock Division Factor. 180M*/
    /*!< Nominal clock prescaler.20M */
    /*!< Nominal Re-sync Jump Width. */
    /*!< Nominal Phase Segment 1. */
     /*!< Nominal Phase Segment 2. */
       /*!< Nominal Propagation Segment. */

       /*!< Data clock prescaler. 180M*/
        /*!< Data Re-sync Jump Width. */
       /*!< Data Phase Segment 1. */
         /*!< Data Phase Segment 2. */
          /*!< Data Propagation Segment. */
		/*
		timingConfig.preDivider=0x03;         
    timingConfig.nominalPrescaler=0x6;  
    timingConfig.nominalRJumpwidth=4; 
    timingConfig.nominalPhaseSeg1=0xF;  
    timingConfig.nominalPhaseSeg2=4;  
    timingConfig.nominalPropSeg=0;    
#ifdef USE_FD
    timingConfig.dataPrescaler=1;     
    timingConfig.dataRJumpwidth=4;    
    timingConfig.dataPhaseSeg1=9;     
    timingConfig.dataPhaseSeg2=2;     
    timingConfig.dataPropSeg=0;       
#endif*/
	
		timingConfig.preDivider=0x01;         
    timingConfig.nominalPrescaler=0x9;  
    timingConfig.nominalRJumpwidth=10; 
    timingConfig.nominalPhaseSeg1=29;  
    timingConfig.nominalPhaseSeg2=10;  
    timingConfig.nominalPropSeg=0;    
#ifdef USE_FD
    timingConfig.dataPrescaler=9;     
    timingConfig.dataRJumpwidth=1;    
    timingConfig.dataPhaseSeg1=2;     
    timingConfig.dataPhaseSeg2=1;     
    timingConfig.dataPropSeg=0;       
#endif


		CAN_SetTimingConfig(base, &timingConfig);
    /* set base address */
    base->MRBA = config->baseAddress;

    /* define 11-bit filters */
    if (CAN_NUM_STANDARD_FILTERS > 0)
    {
        base->SIDFC = ((CAN_NUM_STANDARD_FILTERS & 0xFF) << 16) | (s_ramOffset & 0xFFFF);
        memset((uint32_t *)(base->MRBA + s_ramOffset), 0x00, CAN_NUM_STANDARD_FILTERS * sizeof(CAN_STDFILTER_T));
        s_ramOffset += CAN_NUM_STANDARD_FILTERS * sizeof(CAN_STDFILTER_T);
    }

    /* define 29-bit filters */
    if (CAN_NUM_EXTENDED_FILTERS > 0)
    {
        base->XIDFC = ((CAN_NUM_EXTENDED_FILTERS & 0x7F) << 16) | (s_ramOffset & 0xFFFF);
        memset((uint32_t *)(base->MRBA + s_ramOffset), 0x00, CAN_NUM_EXTENDED_FILTERS * sizeof(CAN_EXTFILTER_T));
        s_ramOffset += CAN_NUM_EXTENDED_FILTERS * sizeof(CAN_EXTFILTER_T);
    }

    /* set number of tx message buffers */
    base->TXBC = ((CAN_NUM_TX_MESSAGE_BUFFERS & 0x3F) << 16) + (s_ramOffset & 0xFFFF);
    /* set tx buffer elements size */
    base->TXESC = CAN_NumberofBytestoFieldSize(CAN_MAX_MESSAGE_BYTES);
    memset((uint32_t *)(base->MRBA + s_ramOffset), 0x00, CAN_NUM_TX_MESSAGE_BUFFERS * sizeof(CAN_BUF_T));
    s_ramOffset += CAN_NUM_TX_MESSAGE_BUFFERS * sizeof(CAN_BUF_T);

    /* set location of rx message buffers */
    base->RXBC = s_ramOffset & 0xFFFF;
    /* set rx message buffer elements size */
    base->RXESC = CAN_NumberofBytestoFieldSize(CAN_MAX_MESSAGE_BYTES) << 8;
    memset((uint32_t *)(base->MRBA + s_ramOffset), 0x00, CAN_NUM_RX_MESSAGE_BUFFERS * sizeof(CAN_BUF_T));
    s_ramOffset += CAN_NUM_RX_MESSAGE_BUFFERS * sizeof(CAN_BUF_T);

    /* default configuration is to reject all non-matching frames */
    base->GFC = CAN_GFC_ANFS_MASK | CAN_GFC_ANFE_MASK;

    /* RTR handling */
    if (config->rejectStandardRTR) base->GFC |= CAN_GFC_RRFS_MASK;
    if (config->rejectExtendedRTR) base->GFC |= CAN_GFC_RRFE_MASK;

    /* enable interrupt 0, all interrupts use interrupt 0 */
    base->ILS = 0;
    base->ILE = CAN_ILE_EINT0_MASK;

#include "LPC54608.h"                   // Device header
    if (config->enableLoopBack)
    {
        base->CCCR |= CAN_CCCR_TEST_MASK;
        base->TEST |= CAN_TEST_LBCK_MASK;
    }

#ifdef USE_FD
    if (config->enableNonISOMode)
    {
        base->CCCR |= CAN_CCCR_NISO_MASK;
    }
#endif
    
    if (config->timestampClock_Hz)
    {
        /* use external timestamp counter */
        base->TSCC = CAN_TSCC_TSS_EXTCOUNTER;
        timestampdivisor = sourceClock_Hz / config->timestampClock_Hz;
        if (timestampdivisor > 0x800) timestampdivisor = 0x800;
        base->ETSCC = CAN_TCTRL_ETCE | (timestampdivisor - 1);
    }
}

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
void CAN_SetRxFifoConfig(CAN_Type *base, uint8_t fifoIdx, const can_rx_fifo_config_t *config, bool enable)
{
    if (fifoIdx >= CAN_NUM_RX_FIFOS) return;

    /* remember FIFO size */
    s_rxFIFOSize[fifoIdx] = config->idFilterNum & 0x7F;
    if (s_rxFIFOSize[fifoIdx] > MAX_RX_FIFO_SIZE) s_rxFIFOSize[fifoIdx] = MAX_RX_FIFO_SIZE;

    switch (fifoIdx)
    {
        case 0:
            if (enable)
            {
                /* set size of Rx FIFO, set offset, blocking mode */
                base->RXF0C = (s_rxFIFOSize[fifoIdx] << 16) | (s_ramOffset & 0xFFFF);
                base->RXESC &= ~CAN_RXESC_F0DS_MASK;
                base->RXESC |= CAN_NumberofBytestoFieldSize(CAN_MAX_MESSAGE_BYTES);
                s_ramOffset += s_rxFIFOSize[fifoIdx] * sizeof(CAN_BUF_T);
            }
            else
            {
                base->RXF0C = 0;
            }
            break;

        case 1:
            if (enable)
            {
                /* set size of Rx FIFO 1, set offset, blocking mode */
                base->RXF1C = (s_rxFIFOSize[fifoIdx] << 16) | (s_ramOffset & 0xFFFF);
                base->RXESC &= ~CAN_RXESC_F1DS_MASK;
                base->RXESC |= (CAN_NumberofBytestoFieldSize(CAN_MAX_MESSAGE_BYTES) << 4);
                s_ramOffset += s_rxFIFOSize[fifoIdx] * sizeof(CAN_BUF_T);
            }
            else
            {
                base->RXF1C = 0;
            }
            break;
    }
}

/*!
 * @brief De-initializes a CAN instance.
 *
 * This function disable the CAN module clock and set all register value
 * to reset value.
 *
 * @param base CAN peripheral base address.
 */
void CAN_Deinit(CAN_Type *base)
{
    CAN_Enable(base, false);

    /* clock off */
    switch (CAN_GetInstance(base))
    {
        case 0:
            CLOCK_DisableClock(kCLOCK_Mcan0);
            break;

        case 1:
            CLOCK_DisableClock(kCLOCK_Mcan1);
            break;
    }
}

/*!
 * @brief Aborts transmission on a CAN transmit message buffer.
 *
 * This function aborts the previous transmission and cleans the Message Buffer
 *
 * @param base CAN peripheral base address.
 * @param mbIdx The Message Buffer index.
 */
void CAN_AbortTxMb(CAN_Type *base, uint8_t mbIdx)
{
    if (mbIdx >= CAN_NUM_TX_MESSAGE_BUFFERS) return;

    /* abort any current transmission in this message buffer */
    if (base->TXBRP & (1UL << mbIdx))
    {
        base->TXBCR |= (1UL << mbIdx);
        while (base->TXBRP & (1UL << mbIdx));
    }
}

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
status_t CAN_WriteTxMb(CAN_Type *base, uint8_t mbIdx, const can_frame_t *txFrame)
{
    CAN_BUF_T *pTxBuffer;
    int b;

    if (mbIdx >= CAN_NUM_TX_MESSAGE_BUFFERS) return kStatus_Fail;

    /* transmission is pending in this message buffer */
    if (base->TXBRP & (1UL << mbIdx)) return kStatus_Fail;

    pTxBuffer = (CAN_BUF_T *)(base->MRBA + (base->TXBC & 0xFFFF) + (mbIdx * sizeof(CAN_BUF_T)));

    if (txFrame->format == kCAN_FrameFormatExtend)
    {
        pTxBuffer->ID = CAN_BUF_ID_XTD | (txFrame->id & 0x1FFFFFFF);
    }
    else
    {
        pTxBuffer->ID = (txFrame->id & 0x7FF) << 18;
    }

    if (txFrame->type == kCAN_FrameTypeRemote) pTxBuffer->ID |= CAN_BUF_ID_RTR;

    pTxBuffer->CONFIG = (CAN_EncodeDLC(txFrame->length) << 16);
#ifdef USE_FD
    if (txFrame->proto == kCAN_ProtoTypeFD) pTxBuffer->CONFIG |= CAN_BUF_CONFIG_FDF;
    if (txFrame->bitratemode == kCAN_BitrateModeTypeSwitch) pTxBuffer->CONFIG |= CAN_BUF_CONFIG_BRS;
#endif
    
    for (b = 0; b < (int)(txFrame->length + (4 - 1)) / 4; b++)
    {
        pTxBuffer->DATA32[b] = txFrame->dataWord[b];
    }

    base->TXBAR = (1 << mbIdx);

    return kStatus_Success;
}

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
status_t CAN_TransferSendBlocking(CAN_Type *base, uint8_t mbIdx, can_frame_t *txFrame)
{
    /* write the message to the message buffer */
    status_t status = CAN_WriteTxMb(base, mbIdx, txFrame);
    if (status == kStatus_Success)
    {
        /* wait for completion */
        while (!(base->TXBRP & (1UL << mbIdx)));
    }

    return status;
}

/*!
 * @brief Sets the CAN receive global mask.
 *
 * This function sets the global mask for CAN in a matching process.
 *
 * @param base CAN peripheral base address.
 * @param filter Rx Global Mask value.
 */
void CAN_SetRxGlobalMask(CAN_Type *base, uint32_t filter)
{
    base->GFC &= (CAN_GFC_RRFS_MASK | CAN_GFC_RRFE_MASK);
    base->GFC |= (filter & (CAN_GFC_ANFS_MASK | CAN_GFC_ANFE_MASK));
}

/*!
 * @brief Sets the CAN receive individual mask.
 *
 * This function sets the individual mask for CAN matching process.
 *
 * @param base CAN peripheral base address.
 * @param maskIdx The Index of individual Mask.
 * @param filter Rx Individual filter value.
 */
void CAN_SetRxIndividualMask(CAN_Type *base, uint8_t maskIdx, uint32_t filter)
{
    CAN_STDFILTER_T *pFilter;

    /* ignore if index is too high */
    if (maskIdx >= CAN_NUM_STANDARD_FILTERS) return;

    pFilter = (CAN_STDFILTER_T *)(base->MRBA + (base->SIDFC & 0xFFFF) + (maskIdx * sizeof(CAN_STDFILTER_T)));
    pFilter->VALUE = filter;
}

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
void CAN_SetRxExtIndividualMask(CAN_Type *base, uint8_t maskIdx, uint32_t filterLow, uint32_t filterHigh)
{
    CAN_EXTFILTER_T *pFilter;

    /* ignore if index is too high */
    if (maskIdx >= CAN_NUM_EXTENDED_FILTERS) return;

    pFilter = (CAN_EXTFILTER_T *)(base->MRBA + (base->XIDFC & 0xFFFF) + (maskIdx * sizeof(CAN_EXTFILTER_T)));
    pFilter->LOWVALUE = filterLow;
    pFilter->HIGHVALUE = filterHigh;
}

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
status_t CAN_ReadRxFifo(CAN_Type *base, uint8_t fifoIdx, can_frame_t *rxFrame)
{
    CAN_BUF_T *pRxBuffer;
    uint8_t GetIndex;
    status_t status = kStatus_Fail;
    __I uint32_t *pRXFS;
    __IO uint32_t *pRXFC, *pRXFA;
    uint8_t msgLostBit;

    /* check for valid FIFO number */
    if (fifoIdx < CAN_NUM_RX_FIFOS)
    {
        if (fifoIdx == 0)
        {
            pRXFS = &(base->RXF0S);
            pRXFC = &(base->RXF0C);
            pRXFA = &(base->RXF0A);
            msgLostBit = 3;
        }
        else
        {
            pRXFS = &(base->RXF1S);
            pRXFC = &(base->RXF1C);
            pRXFA = &(base->RXF1A);
            msgLostBit = 7;
        }

        /* if FIFO is not empty */
        if ((*pRXFS & 0x7F) > 0)
        {
            GetIndex = (uint8_t)((*pRXFS >> 8) & 0x3F);
            pRxBuffer = (CAN_BUF_T *)(base->MRBA + (*pRXFC & 0xFFFF) + (GetIndex * sizeof(CAN_BUF_T)));

            CAN_ReadRxMessage(pRxBuffer, rxFrame);

            /* we got the message */
            *pRXFA = GetIndex;

            /* check for overflow */
            if (*pRXFS & CAN_RXFS_RFL)
            {
                /* clear overflow flag */
                base->IR = (1UL << msgLostBit);

                status = kStatus_CAN_RxOverflow;
            }
            else
            {
                status = kStatus_Success;
            }
        }
    }

    return status;
}

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
status_t CAN_TransferReceiveFifoBlocking(CAN_Type *base, uint8_t fifoIdx, can_frame_t *rxFrame)
{
    status_t status;

    /* check if FIFO is enabled */
    if (fifoIdx == 0)
    {
        if (((base->RXF0C >> 16) & 0x7F) == 0) return kStatus_Fail;
    }
    else
    {
        if (((base->RXF1C >> 16) & 0x7F) == 0) return kStatus_Fail;
    }

    while ((status = CAN_ReadRxFifo(base, fifoIdx, rxFrame)) == kStatus_Fail);

    return status;
}

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
status_t CAN_ReadRxMb(CAN_Type *base, uint8_t mbIdx, can_frame_t *rxFrame)
{
    CAN_BUF_T *pRxBuffer;
    status_t status = kStatus_Fail;
    bool newData = false;

    if (mbIdx < CAN_NUM_RX_MESSAGE_BUFFERS)
    {
        if (mbIdx < 32)
            newData = (base->NDAT1 >> mbIdx) & 1;
        else
            newData = (base->NDAT2 >> (mbIdx - 32)) & 1;

        /* new message is waiting to be read */
        if (newData)
        {
            /* get memory location of rx buffer */
            pRxBuffer = (CAN_BUF_T *)(base->MRBA + (base->RXBC & 0xFFFF) + (mbIdx * sizeof(CAN_BUF_T)));

            /* read the message */
            CAN_ReadRxMessage(pRxBuffer, rxFrame);

            /* clear 'new data' flag */
            if (mbIdx < 32)
                base->NDAT1 |= (1UL << mbIdx);
            else
                base->NDAT2 |= (1UL << (mbIdx - 32));

            status = kStatus_Success;
        }
    }

    return status;
}

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
status_t CAN_TransferReceiveBlocking(CAN_Type *base, uint8_t mbIdx, can_frame_t *rxFrame)
{
    while (CAN_ReadRxMb(base, mbIdx, rxFrame) != kStatus_Success);

    return kStatus_Success;
}

/*!
 * @brief Aborts the interrupt driven message receive process.
 *
 * This function aborts the interrupt driven message receive process.
 *
 * @param base CAN peripheral base address.
 * @param handle CAN handle pointer.
 * @param mbIdx The CAN Message Buffer index.
 */
void CAN_TransferAbortReceive(CAN_Type *base, can_handle_t *handle, uint8_t mbIdx)
{
    /* Disable Message Buffer Interrupt. */
    CAN_DisableInterrupts(base, kCAN_RxBufferInterruptEnable);

    handle->rxMbState[mbIdx] = kCAN_StateIdle;
}

/*!
 * @brief Aborts the interrupt driven message receive from Rx FIFO process.
 *
 * This function aborts the interrupt driven message receive from Rx FIFO process.
 *
 * @param base CAN peripheral base address.
 * @param fifoIdx Number of FIFO, 0 or 1
 * @param handle CAN handle pointer.
 */
void CAN_TransferAbortReceiveFifo(CAN_Type *base, uint8_t fifoIdx, can_handle_t *handle)
{
  if (fifoIdx < CAN_NUM_RX_FIFOS)
  {
      if (fifoIdx == 0)
          CAN_DisableInterrupts(base, kCAN_RxFIFO0InterruptEnable);
      else if (fifoIdx == 1)
          CAN_DisableInterrupts(base, kCAN_RxFIFO0InterruptEnable);

      handle->rxFifoState[fifoIdx] = kCAN_StateIdle;
  }
}

/*!
 * @brief Aborts the interrupt driven message send process.
 *
 * This function aborts the interrupt driven message send process.
 *
 * @param base CAN peripheral base address.
 * @param handle CAN handle pointer.
 * @param mbIdx The CAN Message Buffer index.
 */
void CAN_TransferAbortSend(CAN_Type *base, can_handle_t *handle, uint8_t mbIdx)
{
    CAN_DisableMbInterrupts(base, 1UL << mbIdx);

    handle->txMbState[mbIdx] = kCAN_StateIdle;
}

/*!
 * @brief CAN IRQ handle function.
 *
 * This function handles the CAN Error, the Message Buffer, and the Rx FIFO IRQ request.
 *
 * @param base CAN peripheral base address.
 * @param handle CAN handle pointer.
 */
void CAN_TransferHandleIRQ(CAN_Type *base, can_handle_t *handle)
{
    status_t status = kStatus_CAN_UnHandled;
    uint32_t result;
    bool newdata;
    uint8_t index = 0;

    while (base->IR & base->IE)
    {
        /* error */
        if (base->IR & (kCAN_BusOffInterruptEnable | kCAN_ErrorInterruptEnable | kCAN_WarningInterruptEnable))
        {
            status = kStatus_CAN_ErrorStatus;

            /* clear interrupt flags */
            base->IR = (kCAN_BusOffInterruptEnable | kCAN_ErrorInterruptEnable | kCAN_WarningInterruptEnable);
        }

        /* transmission finished */
        else if (base->IR & kCAN_TxCompletedInterruptEnable)
        {
        	/* look at all message buffers */
            for (index = 0; index < CAN_NUM_TX_MESSAGE_BUFFERS; index++)
            {
            	/* if this message buffer has completed transmission */
                if (((base->TXBTO >> index) & 1) && (handle->txMbState[index] != kCAN_StateIdle))
                {
                    switch (handle->txMbState[index])
                    {
                        case kCAN_StateTxData:
                            status = kStatus_CAN_TxIdle;
                            CAN_TransferAbortSend(base, handle, index);
                            result = index;
                            break;

                        case kCAN_StateTxRemote:
                            CAN_TransferAbortSend(base, handle, index);
                            handle->txMbState[index] = kCAN_StateIdle;
                            handle->rxMbState[index] = kCAN_StateRxRemote;
                            status = kStatus_CAN_TxSwitchToRx;
                            result = index;
                            /* Enable Message Buffer Interrupt. */
                            CAN_EnableInterrupts(base, kCAN_RxBufferInterruptEnable);
                            break;

                        default:
                            status = kStatus_CAN_UnHandled;
                            CAN_TransferAbortSend(base, handle, index);
                            break;
                    }

                    /* Calling Callback Function if has one. */
                    if (handle->callback != NULL)
                    {
                        handle->callback(base, handle, status, result, handle->userData);
                    }

                }
            }

            /* clear interrupt flag */
            base->IR = kCAN_TxCompletedInterruptEnable;
        }

        /* rx message buffer message received */
        else if (base->IR & kCAN_RxBufferInterruptEnable)
        {
            /* get lowest rx message buffer with data waiting to be read */
            for (index = 0; index < CAN_NUM_RX_MESSAGE_BUFFERS; index++)
            {
                if (index < 32)
                    newdata = (base->NDAT1 >> index) & 1;
                else
                    newdata = (base->NDAT2 >> (index - 32)) & 1;

                if (newdata) break;
            }
            /* no data received */
            if (index == CAN_NUM_TX_MESSAGE_BUFFERS)
            {
                break;
            }

            switch (handle->rxMbState[index])
            {
                case kCAN_StateRxData:
                case kCAN_StateRxRemote:
                    status = CAN_ReadRxMb(base, index, handle->mbFrameBuf[index]);
                    if (kStatus_Success == status)
                    {
                        status = kStatus_CAN_RxIdle;
                    }
                    CAN_TransferAbortReceive(base, handle, index);
                    result = index;
                    break;

                default:
                    status = kStatus_CAN_UnHandled;
                    CAN_TransferAbortReceive(base, handle, index);
                    break;
            }

            /* Calling Callback Function if has one. */
            if (handle->callback != NULL)
            {
                handle->callback(base, handle, status, result, handle->userData);
            }

            /* clear interrupt flag */
            base->IR = kCAN_RxBufferInterruptEnable;
        }

        /* rx fifo 0 message received */
        else if (base->IR & kCAN_RxFIFO0InterruptEnable)
        {
            index = 0;
            status = CAN_ReadRxFifo(base, index, handle->rxFifoFrameBuf[index]);
            if (kStatus_Success == status)
            {
                status = kStatus_CAN_RxFifoIdle;
            }
            CAN_TransferAbortReceiveFifo(base, index, handle);
            result = index;

            /* Calling Callback Function if has one. */
            if (handle->callback != NULL)
            {
                handle->callback(base, handle, status, result, handle->userData);
            }

            /* clear interrupt flag */
            base->IR = kCAN_RxFIFO0InterruptEnable;
        }

        /* rx fifo 1 message received */
        else if (base->IR & kCAN_RxFIFO1InterruptEnable)
        {
            index = 1;
            status = CAN_ReadRxFifo(base, index, handle->rxFifoFrameBuf[index]);
            if (kStatus_Success == status)
            {
                status = kStatus_CAN_RxFifoIdle;
            }
            CAN_TransferAbortReceiveFifo(base, index, handle);
            result = index;

            /* Calling Callback Function if has one. */
            if (handle->callback != NULL)
            {
                handle->callback(base, handle, status, result, handle->userData);
            }

            /* clear interrupt flag */
            base->IR = kCAN_RxFIFO1InterruptEnable;
        }

        /* Reset return status */
        status = kStatus_CAN_UnHandled;
    }
}

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
status_t CAN_TransferSendNonBlocking(CAN_Type *base, can_handle_t *handle, can_mb_transfer_t *xfer)
{
    if (xfer->mbIdx < CAN_NUM_TX_MESSAGE_BUFFERS)
    {
        /* Check if Message Buffer is idle. */
        if (kCAN_StateIdle == handle->txMbState[xfer->mbIdx])
        {
            /* Distinguish transmit type. */
            if (kCAN_FrameTypeRemote == xfer->frame->type)
            {
                handle->txMbState[xfer->mbIdx] = kCAN_StateTxRemote;

                /* Register user Frame buffer to receive remote Frame. */
                handle->mbFrameBuf[xfer->mbIdx] = xfer->frame;
            }
            else
            {
                handle->txMbState[xfer->mbIdx] = kCAN_StateTxData;
            }

            /* Enable Message Buffer tx Interrupt. */
            CAN_EnableMbInterrupts(base, 1UL << xfer->mbIdx);

            base->GFC &= ~CAN_GFC_RRFS_MASK;
            if (kStatus_Success == CAN_WriteTxMb(base, xfer->mbIdx, xfer->frame))
            {
                return kStatus_Success;
            }
            else
            {
                /* Disable Message Buffer tx Interrupt. */
                CAN_DisableMbInterrupts(base, 1 << xfer->mbIdx);

                handle->txMbState[xfer->mbIdx] = kCAN_StateIdle;
                return kStatus_Fail;
            }
        }
        else
        {
            return kStatus_CAN_TxBusy;
        }
    }
    else
    {
        return kStatus_Fail;
    }
}

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
status_t CAN_TransferReceiveNonBlocking(CAN_Type *base, can_handle_t *handle, can_mb_transfer_t *xfer)
{
    /* Check if Message Buffer is idle. */
    if (kCAN_StateIdle == handle->rxMbState[xfer->mbIdx])
    {
        handle->rxMbState[xfer->mbIdx] = kCAN_StateRxData;

        /* Register Message Buffer. */
        handle->mbFrameBuf[xfer->mbIdx] = xfer->frame;

        /* Enable Message Buffer Interrupt. */
        CAN_EnableInterrupts(base, kCAN_RxBufferInterruptEnable);

        return kStatus_Success;
    }
    else
    {
        return kStatus_CAN_RxBusy;
    }
}

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
                                                can_fifo_transfer_t *xfer)
{
    if (xfer->fifoIdx < CAN_NUM_RX_FIFOS)
    {
        /* Check if FIFO is idle. */
        if (kCAN_StateIdle == handle->rxFifoState[xfer->fifoIdx])
        {
            handle->rxFifoState[xfer->fifoIdx] = kCAN_StateRxFifo;

            /* Register Message Buffer. */
            handle->rxFifoFrameBuf[xfer->fifoIdx] = xfer->frame;

            if (xfer->fifoIdx == 0)
            {
                /* Enable rx FIFO 0 Interrupt. */
                CAN_EnableInterrupts(base, kCAN_RxFIFO0InterruptEnable);
            }
            else if (xfer->fifoIdx == 1)
            {
                /* Enable rx FIFO 0 Interrupt. */
                CAN_EnableInterrupts(base, kCAN_RxFIFO1InterruptEnable);
            }

            return kStatus_Success;
        }
        else
        {
            return kStatus_CAN_RxFifoBusy;
        }
    }
    else
    {
        return kStatus_Fail;
    }
}

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
                                  void *userData)
{
    uint8_t instance;

    /* Clean CAN transfer handle. */
    memset(handle, 0, sizeof(*handle));

    /* Get instance from peripheral base address. */
    instance = CAN_GetInstance(base);

    /* Save the context in global variables to support the double weak mechanism. */
    s_canHandle[instance] = handle;

    /* Register Callback function. */
    handle->callback = callback;
    handle->userData = userData;

    /* We Enable Error & Status interrupt here, because this interrupt just
     * report current status of CAN module through Callback function.
     * It is insignificance without a available callback function.
     */
    if (handle->callback != NULL)
    {
        CAN_EnableInterrupts(base, kCAN_BusOffInterruptEnable | kCAN_ErrorInterruptEnable |
          kCAN_WarningInterruptEnable | kCAN_TxCompletedInterruptEnable);
    }
    else
    {
        CAN_DisableInterrupts(base, kCAN_BusOffInterruptEnable | kCAN_ErrorInterruptEnable |
          kCAN_WarningInterruptEnable | kCAN_TxCompletedInterruptEnable);
    }
}

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
void CAN_GetDefaultConfig(can_config_t *config)
{
    memset(config, 0, sizeof(can_config_t));
    config->nominalBaudRate   = 125000;
#ifdef USE_FD
    config->dataBaudRate      = 0;
#endif
    config->baseAddress       = 0x20010000;
    config->timestampClock_Hz = 0;
    config->rejectStandardRTR = true;
    config->rejectExtendedRTR = true;
    config->enableLoopBack    = false;
#ifdef USE_FD
    config->enableNonISOMode  = false;
    config->disableFD         = false;
#endif
}

#if (FSL_FEATURE_SOC_CAN_COUNT > 0)
/*!
 * @brief CAN controller 0 interrupt 0
 */
void CAN0_IRQ0_DriverIRQHandler(void)
{
    CAN_TransferHandleIRQ(CAN0, s_canHandle[0]);
}
#endif

#if (FSL_FEATURE_SOC_CAN_COUNT > 1)
/*!
 * @brief CAN controller 1 interrupt 0
 */
void CAN1_IRQ0_DriverIRQHandler(void)
{
    CAN_TransferHandleIRQ(CAN1, s_canHandle[1]);
}
#endif
