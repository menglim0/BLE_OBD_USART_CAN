/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "can.h"
#include "fsl_usart.h"
#include "obd_can.h"

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions For GPIO
 ******************************************************************************/

#define APP_BOARD_TEST_GPIO_PORT1 BOARD_LED3_GPIO_PORT
#define APP_BOARD_TEST_GPIO_PORT2 BOARD_LED1_GPIO_PORT
#define APP_BOARD_TEST_GPIO_PORT3 BOARD_LED2_GPIO_PORT
#define APP_BOARD_TEST_LED1_PIN BOARD_LED3_GPIO_PIN
#define APP_BOARD_TEST_LED2_PIN BOARD_LED1_GPIO_PIN
#define APP_BOARD_TEST_LED3_PIN BOARD_LED2_GPIO_PIN

/*******************************************************************************
 * Definitions for USART
 ******************************************************************************/
#define DEMO_USART USART0
#define DEMO_USART_CLK_SRC kCLOCK_Flexcomm0
#define DEMO_USART_CLK_FREQ CLOCK_GetFreq(kCLOCK_Flexcomm0)
#define DEMO_USART_IRQn FLEXCOMM0_IRQn
#define DEMO_USART_IRQHandler FLEXCOMM0_IRQHandler

/*! @brief Ring buffer size (Unit: Byte). */
#define DEMO_RING_BUFFER_SIZE 16

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

#define TICKRATE_HZ (1000)	          /* 1000 ticks per second */
#define TRANSMIT_PERIOD (500)         /* milliseconds between transmission */
#define KeepAlive_PERIOD (2000)         /* milliseconds between transmission */

static volatile uint32_t gTimCnt = 0; /* incremented every millisecond */

volatile uint16_t txIndex; /* Index of the data to send out. */
volatile uint16_t rxIndex; /* Index of the memory to save new arrived data. */
uint16_t rxIndex_old,rxIndex_count,rxIndex_loop,delay_count,debug_count;
bool rxIndex_updated,tx_CAN_Enable,message_received,Keep_Service_Active;

uint8_t VfCANH_RxMSG_Data;
uint16_t VfCANH_RxMSG_ID;

uint8_t VfUSART_Data[12];
uint8_t USART_Data[DEMO_RING_BUFFER_SIZE],i;
uint8_t demoRingBuffer[DEMO_RING_BUFFER_SIZE];
uint8_t ReceiveDataFromCAN_to_USART[12];

usart_handle_t usart0_Define;

uint8_t g_tipString[] =
    "Usart functional API interrupt example\r\nBoard receives characters then sends them out\r\nNow please input:\r\n";


/*******************************************************************************
 * Code
 ******************************************************************************/
 void DEMO_USART_IRQHandler(void)
{
    uint8_t data;
		rxIndex_updated=false;
		delay_count=0;
    /* If new data arrived. */
    if ((kUSART_RxFifoNotEmptyFlag | kUSART_RxError) & USART_GetStatusFlags(DEMO_USART))
    {
			
        data = USART_ReadByte(DEMO_USART);
        /* If ring buffer is not full, add data to ring buffer. */
        //if (((rxIndex + 1) % DEMO_RING_BUFFER_SIZE) != txIndex)
        {
            demoRingBuffer[rxIndex] = data;
            rxIndex++;
					rxIndex_count++;
            rxIndex %= DEMO_RING_BUFFER_SIZE;
					
        }
				  //USART_DisableInterrupts(DEMO_USART, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable);
    }
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
 
 

/*!
 * @brief Keeps track of time
 */
void SysTick_Handler(void)
{
	// count milliseconds
	gTimCnt++;
}

/*!
 * @brief Main function
 */
int main(void)
{
    usart_config_t config;
    can_frame_t txmsg = { 0 };
		
		can_frame_t Alive_msg_3E = { 0 };
		
		Alive_msg_3E.id=0x14dae1f1;
		Alive_msg_3E.dataByte[0]=0x02;
		Alive_msg_3E.dataByte[1]=0x3E;
		Alive_msg_3E.format =kCAN_FrameFormatExtend;
		Alive_msg_3E.type = kCAN_FrameTypeData;
		Alive_msg_3E.proto = kCAN_ProtoTypeClassic;
		Alive_msg_3E.bitratemode = kCAN_BitrateModeTypeSwitch;
		Alive_msg_3E.length = 8;
		
		txmsg.dataByte[0] = 0xFC;
		txmsg.dataByte[1] = 0x56;
    can_frame_t rxmsg = { 0 };
    int b;
    bool message_transmitted = false;
    uint32_t next_id = 0x4C8;

    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput, 0,
    };

    /* Board pin, clock, debug console init */
		

		
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);
    CLOCK_EnableClock(kCLOCK_Gpio2);
    CLOCK_EnableClock(kCLOCK_Gpio3);

	/* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
		
    BOARD_InitPins();
    BOARD_BootClockFROHF48M();
    BOARD_InitDebugConsole();
		BOARD_InitCAN();
		
		
		//USART_EnableInterrupts(USART1, kUSART_TxLevelInterruptEnable | kUSART_RxLevelInterruptEnable);
    /* print a note to terminal */
    PRINTF("\r\n CAN-FD driver message objects example\r\n");
   
    /* Enable SysTick Timer */
    SysTick_Config(SystemCoreClock / TICKRATE_HZ);
				
		 /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUSART_ParityDisabled;
     * config.stopBitCount = kUSART_OneStopBit;
     * config.loopback = false;
     * config.enableTxFifo = false;
     * config.enableRxFifo = false;
     */
    USART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;

    USART_Init(DEMO_USART, &config, DEMO_USART_CLK_FREQ);
    /* Send g_tipString out. */
  //  USART_WriteBlocking(DEMO_USART, g_tipString, sizeof(g_tipString) / sizeof(g_tipString[0]));
			
   /* Enable RX interrupt. */
    USART_EnableInterrupts(DEMO_USART, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable);
    EnableIRQ(DEMO_USART_IRQn);

    /* Init output LED GPIO. */
		
    GPIO_PinInit(GPIO, BOARD_LED1_GPIO_PORT, BOARD_LED1_GPIO_PIN, &led_config);
    GPIO_WritePinOutput(GPIO, BOARD_LED1_GPIO_PORT, BOARD_LED1_GPIO_PIN, 1);
    GPIO_PinInit(GPIO, BOARD_LED2_GPIO_PORT, BOARD_LED2_GPIO_PIN, &led_config);
    GPIO_WritePinOutput(GPIO, BOARD_LED2_GPIO_PORT, BOARD_LED2_GPIO_PIN, 1);
    GPIO_PinInit(GPIO, BOARD_LED3_GPIO_PORT, BOARD_LED3_GPIO_PIN, &led_config);
    GPIO_WritePinOutput(GPIO, BOARD_LED3_GPIO_PORT, BOARD_LED3_GPIO_PIN, 1);
		
		/*		init the control for CC2540 on the OBD module*/
		/*
		GPIO_PinInit(GPIO, BOARD_CC2540_EN_GPIO0, BOARD_CC2540_EN_GPIO_PIN, &led_config);
		GPIO_WritePinOutput(GPIO, BOARD_CC2540_EN_GPIO0, BOARD_CC2540_EN_GPIO_PIN,0);
		
		GPIO_PinInit(GPIO, BOARD_CC2540_EN_GPIO0, BOARD_CC2540_BT_GPIO_PIN, &led_config);
		GPIO_WritePinOutput(GPIO, BOARD_CC2540_EN_GPIO0, BOARD_CC2540_BT_GPIO_PIN,0);
		
		GPIO_PinInit(GPIO, BOARD_CC2540_EN_GPIO0, BOARD_CC2540_BC_GPIO_PIN, &led_config);
		GPIO_WritePinOutput(GPIO, BOARD_CC2540_EN_GPIO0, BOARD_CC2540_BC_GPIO_PIN,0);
		*/
		        txmsg.id = 0x145;
            //txmsg.format = kCAN_FrameFormatStandard;
						txmsg.format =kCAN_FrameFormatExtend;
            txmsg.type = kCAN_FrameTypeData;
            txmsg.proto = kCAN_ProtoTypeClassic;
            txmsg.bitratemode = kCAN_BitrateModeTypeSwitch;
            txmsg.length = 8;

		    /* Send g_tipString out. */
    //USART_WriteBlocking(DEMO_USART, g_tipString, sizeof(g_tipString) / sizeof(g_tipString[0]));
		

    while (true)
    {
			if(rxIndex_updated==true)
			{
				for (rxIndex_loop=0;rxIndex_loop<rxIndex_count;rxIndex_loop++)
				{
					USART_Data[rxIndex_loop]=demoRingBuffer[(rxIndex_old+rxIndex_loop)%16];
					if(rxIndex_loop>=4)
					{
					txmsg.dataByte[rxIndex_loop-4] = USART_Data[rxIndex_loop];
					}
					
				}
				
				/*if 3e receive, keep the 3e alive*/
				if(txmsg.dataByte[1]==0x3e)
				{
				Keep_Service_Active = true;
				
				}
				else if(txmsg.dataByte[1]==0x3f)
				{Keep_Service_Active = false;}
				
					//txmsg.id = ((USART_Data[0]&0x1F)<<24)+(USART_Data[1]<<16)+(USART_Data[2]<<8)+(USART_Data[3]);
					//message_transmitted=obd_can_TxMSG_Standard(CAN0, 0, &txmsg);
			rxIndex_old=rxIndex;
				if(rxIndex_count==12)
				{
					tx_CAN_Enable = true;
				}
			rxIndex_count=0;
				rxIndex_updated=false;
				//tx_CAN_Enable = true;
				
			}
			else
			{
			
				if(delay_count>1000)
				{rxIndex_updated=true;}
				else
				{
					delay_count++;
				}
			}

			/* Send data only when USART TX register is empty and ring buffer has data to send out.
        while ((kUSART_TxFifoNotFullFlag & USART_GetStatusFlags(DEMO_USART)) && (rxIndex != txIndex))
        {
            USART_WriteByte(DEMO_USART, demoRingBuffer[txIndex]);
            txIndex++;
            txIndex %= DEMO_RING_BUFFER_SIZE;
        } 
    */
			
			
        /* check for any received messages on CAN1 message buffer 0 */
        if (CAN_ReadRxMb(CAN0, 0, &rxmsg) == kStatus_Success)
        {
            //PRINTF("Rx buf 0: Received message 0x%3.3X\r\n", rxmsg.id);
					message_received = true;
          	//VfCANH_RxMSG_Data=rxmsg.dataByte[1]; Read the Rx buffer Byte1
					
					VfCANH_RxMSG_ID=rxmsg.id;
					
					if(VfCANH_RxMSG_ID==0x4C9)
					{
						GPIO_TogglePinsOutput(GPIO, BOARD_LED3_GPIO_PORT, 1u << BOARD_LED3_GPIO_PIN);
						
						for(txIndex=0;txIndex<=12;txIndex++)
						{
							debug_count=0;
							if(txIndex>=4)
							{
							ReceiveDataFromCAN_to_USART[txIndex]=rxmsg.dataByte[txIndex-4];
							}
							if(txIndex >= 12)
							{
								message_received = false;
							}
							/* When Receive scuessful, then wait for the CAN Receive data, use the usart send to Host*/
							//while ((kUSART_TxFifoNotFullFlag & USART_GetStatusFlags(DEMO_USART)) )
							{
								
									USART_WriteByte(DEMO_USART, ReceiveDataFromCAN_to_USART[txIndex]);
									//txIndex++;
								debug_count++;

							} 
						}
					}
					//USART_WriteByte(USART0, VfCANH_RxMSG_Data);
					//USART_WriteByte(USART0, 0x00);
            /* toggle LED2 */
				   GPIO_TogglePinsOutput(GPIO, BOARD_LED2_GPIO_PORT, 1u << BOARD_LED2_GPIO_PIN);
        }
				
				

				
				
				/* Transmit the received data here*/
				/***
				
						txmsg.id = byte0-3;
            txmsg.length = 8;
				*/
				
				if ((gTimCnt % TRANSMIT_PERIOD == 0) && !message_transmitted )
				{
					if(tx_CAN_Enable == true)
					{
					txmsg.id = ((USART_Data[0]&0x1F)<<24)+(USART_Data[1]<<16)+(USART_Data[2]<<8)+(USART_Data[3]);
					message_transmitted=obd_can_TxMSG_Standard(CAN0, 0, &txmsg);
					tx_CAN_Enable = false;
					}
					GPIO_TogglePinsOutput(GPIO, BOARD_LED1_GPIO_PORT, 1u << BOARD_LED1_GPIO_PIN);
					 // USART_EnableInterrupts(DEMO_USART, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable);
				}
				else if (gTimCnt % TRANSMIT_PERIOD != 0)
        {
            message_transmitted = false;
        }
		
				if ((gTimCnt % KeepAlive_PERIOD == 0) && Keep_Service_Active )
				{
				obd_can_TxMSG_Standard(CAN0, 0, &Alive_msg_3E);
					//PRINTF("Keep_Alive");
					//debug_count++;
				}
				
				//for(debug_count=0;debug_count<10;debug_count++)
				{;}
    }
}


