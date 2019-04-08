/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * TEXT BELOW IS USED AS SETTING FOR THE PINS TOOL *****************************
PinsProfile:
- !!product 'Pins v2.0'
- !!processor 'LPC54608J512'
- !!package 'LPC54608J512ET180'
- !!mcu_data 'ksdk2_0'
- !!processor_version '0.0.0'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR THE PINS TOOL ***
 */

#include "fsl_common.h"
#include "fsl_iocon.h"
#include "pin_mux.h"

#define IOCON_PIO_DIGITAL_EN        0x0100u   /*!< Enables digital function */
#define IOCON_PIO_FUNC0               0x00u   /*!< Selects pin function 0 */
#define IOCON_PIO_FUNC1               0x01u   /*!< Selects pin function 1 */
#define IOCON_PIO_INPFILT_OFF       0x0200u   /*!< Input filter disabled */
#define IOCON_PIO_INV_DI              0x00u   /*!< Input function is not inverted */
#define IOCON_PIO_MODE_INACT          0x00u   /*!< No addition pin function */
#define IOCON_PIO_MODE_PULLUP         0x20u   /*!< Selects pull-up function */
#define IOCON_PIO_MODE_PULLDown       0x10u   /*!< Selects pull-down function */
#define IOCON_PIO_OPENDRAIN_DI        0x00u   /*!< Open drain is disabled */
#define IOCON_PIO_OPENDRAIN_EN        0x800u   /*!< Open drain is disabled */
#define IOCON_PIO_SLEW_STANDARD       0x00u   /*!< Standard mode, output slew rate control is enabled */
#define PIN1_IDX                         1u   /*!< Pin number for pin 2 in a port 2 */
#define PIN2_IDX                         2u   /*!< Pin number for pin 2 in a port 2 */
#define PIN3_IDX                         3u   /*!< Pin number for pin 3 in a port 3 */
#define PIN4_IDX                         4u   /*!< Pin number for pin 4 in a port 0 */
#define PIN5_IDX                         5u   /*!< Pin number for pin 5 in a port 0 */
#define PIN6_IDX                         6u   /*!< Pin number for pin 6 in a port 0 */
#define PIN14_IDX                       14u   /*!< Pin number for pin 14 in a port 3 */
#define PIN24_IDX                       29u   /*!< Pin number for pin 24 in a port 0 */
#define PIN25_IDX                       29u   /*!< Pin number for pin 25 in a port 0 */
#define PIN26_IDX                       29u   /*!< Pin number for pin 26 in a port 0 */
#define PIN29_IDX                       29u   /*!< Pin number for pin 29 in a port 0 */
#define PIN30_IDX                       30u   /*!< Pin number for pin 30 in a port 0 */
#define PORT0_IDX                        0u   /*!< Port index */
#define PORT1_IDX                        1u   /*!< Port index */
#define PORT2_IDX                        2u   /*!< Port index */
#define PORT3_IDX                        3u   /*!< Port index */

/*
 * TEXT BELOW IS USED AS SETTING FOR THE PINS TOOL *****************************
BOARD_InitPins:
- options: {coreID: singlecore, enableClock: 'true'}
- pin_list:
  - {pin_num: B13, peripheral: FLEXCOMM0, signal: RXD_SDA_MOSI, pin_signal: PIO0_29/FC0_RXD_SDA_MOSI/CTIMER2_MAT3/SCT0_OUT8/TRACEDATA(2), mode: inactive, invert: disabled,
    glitch_filter: disabled, slew_rate: standard, open_drain: disabled}
  - {pin_num: A2, peripheral: FLEXCOMM0, signal: TXD_SCL_MISO, pin_signal: PIO0_30/FC0_TXD_SCL_MISO/CTIMER0_MAT0/SCT0_OUT9/TRACEDATA(1), mode: inactive, invert: disabled,
    glitch_filter: disabled, slew_rate: standard, open_drain: disabled}
  - {pin_num: C8, peripheral: GPIO, signal: 'PIO0, 4', pin_signal: PIO0_4/CAN0_RD/FC4_SCK/CTIMER3_CAP0/SCT0_GPI4/EMC_D(2)/ENET_MDC, mode: pullUp, invert: disabled,
    glitch_filter: disabled, slew_rate: standard, open_drain: disabled}
  - {pin_num: E7, peripheral: GPIO, signal: 'PIO0, 5', pin_signal: PIO0_5/CAN0_TD/FC4_RXD_SDA_MOSI/CTIMER3_MAT0/SCT0_GPI5/EMC_D(3)/ENET_MDIO, mode: pullUp, invert: disabled,
    glitch_filter: disabled, slew_rate: standard, open_drain: disabled}
  - {pin_num: A5, peripheral: GPIO, signal: 'PIO0, 6', pin_signal: PIO0_6/FC3_SCK/CTIMER3_CAP1/CTIMER4_MAT0/SCT0_GPI6/EMC_D(4)/ENET_RX_DV, mode: pullUp, invert: disabled,
    glitch_filter: disabled, slew_rate: standard, open_drain: disabled}
  - {pin_num: C3, peripheral: GPIO, signal: 'PIO2, 2', pin_signal: PIO2_2/ENET_CRS/FC3_SSEL3/SCT0_OUT6/CTIMER1_MAT1, mode: pullUp, invert: disabled, glitch_filter: disabled,
    slew_rate: standard, open_drain: disabled}
  - {pin_num: A13, peripheral: GPIO, signal: 'PIO3, 3', pin_signal: PIO3_3/LCD_VD(17)/FC9_TXD_SCL_MISO, mode: pullUp, invert: disabled, glitch_filter: disabled, slew_rate: standard,
    open_drain: disabled}
  - {pin_num: E3, peripheral: GPIO, signal: 'PIO3, 14', pin_signal: PIO3_14/SCT0_OUT4/FC9_RTS_SCL_SSEL1/CTIMER3_MAT1/TRACEDATA(2), mode: pullUp, invert: disabled,
    glitch_filter: disabled, slew_rate: standard, open_drain: disabled}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR THE PINS TOOL ***
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void BOARD_InitPins(void) {
  CLOCK_EnableClock(kCLOCK_Iocon);                           /* Enables the clock for the IOCON block. 0 = Disable; 1 = Enable.: 0x01u */

	  const uint32_t port0_pin24_config = (
    IOCON_PIO_FUNC0 |                                        /* Pin is configured as FC0_RXD_SDA_MOSI */
    IOCON_PIO_MODE_PULLDown |                                   /* No addition pin function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
  );
  IOCON_PinMuxSet(IOCON, PORT0_IDX, PIN24_IDX, port0_pin24_config); /* PORT0 PIN29 (coords: B13) is configured as FC0_RXD_SDA_MOSI */
		
		const uint32_t port0_pin25_config = (
    IOCON_PIO_FUNC0 |                                        /* Pin is configured as FC0_RXD_SDA_MOSI */
    IOCON_PIO_MODE_PULLDown |                                   /* No addition pin function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
  );
  IOCON_PinMuxSet(IOCON, PORT0_IDX, PIN25_IDX, port0_pin25_config); /* PORT0 PIN25 (coords: B13) is configured as FC0_RXD_SDA_MOSI */
				
		const uint32_t port0_pin26_config = (
    IOCON_PIO_FUNC0 |                                        /* Pin is configured as FC0_RXD_SDA_MOSI */
    IOCON_PIO_MODE_PULLDown |                                   /* No addition pin function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
  );
  IOCON_PinMuxSet(IOCON, PORT0_IDX, PIN26_IDX, port0_pin26_config); /* PORT0 PIN25 (coords: B13) is configured as FC0_RXD_SDA_MOSI */
	
  const uint32_t port0_pin29_config = (
    IOCON_PIO_FUNC1 |                                        /* Pin is configured as FC0_RXD_SDA_MOSI */
    IOCON_PIO_MODE_INACT |                                   /* No addition pin function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
  );
  IOCON_PinMuxSet(IOCON, PORT0_IDX, PIN29_IDX, port0_pin29_config); /* PORT0 PIN29 (coords: B13) is configured as FC0_RXD_SDA_MOSI */
  const uint32_t port0_pin30_config = (
    IOCON_PIO_FUNC1 |                                        /* Pin is configured as FC0_TXD_SCL_MISO */
    IOCON_PIO_MODE_INACT |                                   /* No addition pin function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
  );
  IOCON_PinMuxSet(IOCON, PORT0_IDX, PIN30_IDX, port0_pin30_config); /* PORT0 PIN30 (coords: A2) is configured as FC0_TXD_SCL_MISO */
  const uint32_t port0_pin4_config = (
    IOCON_PIO_FUNC0 |                                        /* Pin is configured as PIO0_4 */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
  );
  IOCON_PinMuxSet(IOCON, PORT0_IDX, PIN4_IDX, port0_pin4_config); /* PORT0 PIN4 (coords: C8) is configured as PIO0_4 */
  const uint32_t port0_pin5_config = (
    IOCON_PIO_FUNC0 |                                        /* Pin is configured as PIO0_5 */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
  );
  IOCON_PinMuxSet(IOCON, PORT0_IDX, PIN5_IDX, port0_pin5_config); /* PORT0 PIN5 (coords: E7) is configured as PIO0_5 */
  const uint32_t port0_pin6_config = (
    IOCON_PIO_FUNC0 |                                        /* Pin is configured as PIO0_6 */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
  );
  IOCON_PinMuxSet(IOCON, PORT0_IDX, PIN6_IDX, port0_pin6_config); /* PORT0 PIN6 (coords: A5) is configured as PIO0_6 */
  const uint32_t port2_pin2_config = (
    IOCON_PIO_FUNC0 |                                        /* Pin is configured as PIO2_2 */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
  );
  IOCON_PinMuxSet(IOCON, PORT2_IDX, PIN2_IDX, port2_pin2_config); /* PORT2 PIN2 (coords: C3) is configured as PIO2_2 */
  const uint32_t port3_pin14_config = (
    IOCON_PIO_FUNC0 |                                        /* Pin is configured as PIO3_14 */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
  );
  IOCON_PinMuxSet(IOCON, PORT3_IDX, PIN14_IDX, port3_pin14_config); /* PORT3 PIN14 (coords: E3) is configured as PIO3_14 */
  const uint32_t port3_pin3_config = (
    IOCON_PIO_FUNC0 |                                        /* Pin is configured as PIO3_3 */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
  );
  IOCON_PinMuxSet(IOCON, PORT3_IDX, PIN3_IDX, port3_pin3_config); /* PORT3 PIN3 (coords: A13) is configured as PIO3_3 */
	
	const uint32_t port1_pin1_config = (
    IOCON_PIO_FUNC0 |                                        /* Pin is configured as PIO0_4 */
    IOCON_PIO_MODE_PULLUP |                                  /* Selects pull-up function */
    IOCON_PIO_INV_DI |                                       /* Input function is not inverted */
    IOCON_PIO_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_PIO_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_PIO_SLEW_STANDARD |                                /* Standard mode, output slew rate control is enabled */
    IOCON_PIO_OPENDRAIN_DI                                   /* Open drain is disabled */
  );
  IOCON_PinMuxSet(IOCON, PORT1_IDX, PIN1_IDX, port1_pin1_config); /* PORT0 PIN4 (coords: C8) is configured as PIO0_4 */

//<<<<<<< Updated upstream
  /* configure P3_18 for CAN0_TX and P3_19 for CAN0_RX */
  //IOCON_PinMuxSet(IOCON, 1, 2, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
  //IOCON_PinMuxSet(IOCON, 1, 3, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
    IOCON_PinMuxSet(IOCON, 3, 18, IOCON_MODE_INACT | IOCON_FUNC4 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
  IOCON_PinMuxSet(IOCON, 3, 19, IOCON_MODE_INACT | IOCON_FUNC4 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
//=======
  /* configure P3_18 for CAN0_TX and P3_19 for CAN0_RX,update here for LPC54616 */
  //IOCON_PinMuxSet(IOCON, 1, 2, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
  //IOCON_PinMuxSet(IOCON, 1, 3, IOCON_MODE_INACT | IOCON_FUNC1 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
   // IOCON_PinMuxSet(IOCON, 3, 18, IOCON_MODE_INACT | IOCON_FUNC4 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
  //IOCON_PinMuxSet(IOCON, 3, 19, IOCON_MODE_INACT | IOCON_FUNC4 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
//>>>>>>> Stashed changes
  /* configure P1_17 for CAN1_TX and P1_18 for CAN1_RX */
  IOCON_PinMuxSet(IOCON, 1, 17, IOCON_MODE_INACT | IOCON_FUNC5 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
  IOCON_PinMuxSet(IOCON, 1, 18, IOCON_MODE_INACT | IOCON_FUNC5 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
