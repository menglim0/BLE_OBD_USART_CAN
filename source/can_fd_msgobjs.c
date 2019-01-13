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
 
 /* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/*  Standard C Included Files */
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "can.h"
#include "fsl_usart.h"
#include "obd_can.h"

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions for RTOS
 ******************************************************************************/
 
#define TOUCHTASK_STACKSIZE 100
#define TOUCHTASK_PRIORITY  (tskIDLE_PRIORITY + 1UL)
#define LCDTASK_PRIORITY  (tskIDLE_PRIORITY + 0UL)
#define LCDTASK_STACKSIZE 100

/**
 * Touch status check delay
 */
#define TOUCH_DELAY   (100)
#define LCD_DELAY   (2000)


static void vTouchTask(void *pvParameters);
static void vLcdTask(void *pvParameters);

void USART_ReceiveData(void);

TaskHandle_t xTouchTaskHandle = NULL;
TaskHandle_t xLcdTaskHandle = NULL;
 
volatile uint32_t g_systickCounter;

TeOBD_Service_MODE OBD_Service_Mode_Detection;	

void SysTick_DelayTicks(uint32_t n)
{
    g_systickCounter = n;
    while(g_systickCounter != 0U)
    {
    }
}
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

static volatile uint32_t gTimCnt = 0,gTimCnt_old; /* incremented every millisecond */

static volatile bool KeepAlive_PERIOD_flag_interrupt;

volatile uint16_t txIndex; /* Index of the data to send out. */
volatile uint16_t rxIndex; /* Index of the memory to save new arrived data. */
uint16_t rxIndex_old,rxIndex_count,rxIndex_loop,delay_count,debug_count;
bool rxIndex_updated,tx_CAN_Enable,message_received,Keep_Service_Active,KeepAlive_PERIOD_flag;

uint8_t VfCANH_RxMSG_Data,KeepSendOneTime;
uint16_t VfCANH_RxMSG_ID,Array_Cycle;

uint8_t VfUSART_Data[DEMO_RING_BUFFER_SIZE];
uint8_t USART_Data[DEMO_RING_BUFFER_SIZE],i;
uint8_t demoRingBuffer[DEMO_RING_BUFFER_SIZE];
uint8_t ReceiveDataFromCAN_to_USART[12];

usart_handle_t usart0_Define;

uint8_t g_tipString[] =
    "Usart functional API interrupt example\r\nBoard receives characters then sends them out\r\nNow please input:\r\n";
		
uint8_t Multi_Frame_Key[2][8]={{0x21,0x20,0x20,0x20,0x20,0x20,0x20,0x20},
																{0x22,0x20,0x00,0x00,0x00,0x00,0x00,0x00}};

																
//按键消息队列的数量
#define KEYMSG_Q_NUM    1  		//按键消息队列的数量  
#define MESSAGE_Q_NUM   4   	//发送数据的消息队列的数量 
QueueHandle_t Key_Queue;   		//按键值消息队列句柄
QueueHandle_t Message_Queue;	//信息队列句柄
#define USART_REC_LEN  			20  	//定义最大接收字节数 50
																
/* Usart queue*/
extern QueueHandle_t Message_Queue;	//信息队列句柄
bool USART_RX_First=0,USART_RX_Last=0,USART_RX_Wait=0;       //接收状态标记
																	

/*******************************************************************************
 * Code
 ******************************************************************************/
 void DEMO_USART_IRQHandler(void)
{
    uint8_t data;
		rxIndex_updated=false;
		delay_count=0;

	
	BaseType_t xHigherPriorityTaskWoken=false;
	
    /* If new data arrived. */
    if ((kUSART_RxFifoNotEmptyFlag | kUSART_RxError) & USART_GetStatusFlags(DEMO_USART))
    {
			
        data = USART_ReadByte(DEMO_USART);
					USART_RX_First = 1;
        /* If ring buffer is not full, add data to ring buffer. */
        //if (((rxIndex + 1) % DEMO_RING_BUFFER_SIZE) != txIndex)
        {
            demoRingBuffer[rxIndex] = data;
            rxIndex++;
						rxIndex_count++;
					if(rxIndex_count>=9)
					{
						USART_RX_Last=1;
					}
            rxIndex %= DEMO_RING_BUFFER_SIZE;
					
        }
				  //USART_DisableInterrupts(DEMO_USART, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable);
    }
		
		//就向队列发送接收到的数据
//	if((USART_RX_STA)&&(Message_Queue!=NULL))
//	{
//		xQueueSendFromISR(Message_Queue,demoRingBuffer,&xHigherPriorityTaskWoken);//向队列中发送数据
//		
//		//demoRingBuffer[0]=0;	
//		memset(demoRingBuffer,0,USART_REC_LEN);//清除数据接收缓冲区USART_RX_BUF,用于下一次数据接收
//	
//		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//如果需要的话进行一次任务切换
//	}
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}
 
 

/*!
 * @brief Keeps track of time
 */
////void SysTick_Handler(void)
//{
//	// count milliseconds
//	gTimCnt++;

//}

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
//    int b;
    bool message_transmitted = false;
    //uint32_t next_id = 0x4C8;

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
		BOARD_InitGPIO();
		
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
    USART_EnableInterrupts(DEMO_USART, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable);
    EnableIRQ(DEMO_USART_IRQn);


		        txmsg.id = 0x145;
            //txmsg.format = kCAN_FrameFormatStandard;
						txmsg.format =kCAN_FrameFormatExtend;
            txmsg.type = kCAN_FrameTypeData;
            txmsg.proto = kCAN_ProtoTypeClassic;
            txmsg.bitratemode = kCAN_BitrateModeTypeSwitch;
            txmsg.length = 8;

		/* scheduler the task here */   		
		xTaskCreate(vTouchTask,"Touch Task",TOUCHTASK_STACKSIZE,NULL,TOUCHTASK_PRIORITY,&xTouchTaskHandle);
	  xTaskCreate(vLcdTask,"LCD Task",LCDTASK_STACKSIZE,NULL,LCDTASK_PRIORITY,&xLcdTaskHandle);
		vTaskStartScheduler();
		
		while(1)
		{
			;
		}
		
    while (0)
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
				{
					Keep_Service_Active = false;
				}
				
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
				{
					rxIndex_updated=true;
				}
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

    }
}

static void vTouchTask(void *pvParameters)
{
	uint8_t key[12];
	//创建消息队列
    //Key_Queue=xQueueCreate(KEYMSG_Q_NUM,sizeof(uint8_t));        //创建消息Key_Queue
    //Message_Queue=xQueueCreate(MESSAGE_Q_NUM,USART_REC_LEN); //创建消息Message_Queue,队列项长度是串口接收缓冲区长度
	for(;;)
	{
		GPIO_TogglePinsOutput(GPIO, BOARD_LED3_GPIO_PORT, 1u << BOARD_LED3_GPIO_PIN);
		KeepSendOneTime=KeepSendOneTime%10;
		//xQueueReceive(Message_Queue,&key,portMAX_DELAY);
		USART_ReceiveData();
		obd_Service(OBD_Service_Mode_Detection);
		KeepSendOneTime++;
		
		vTaskDelay(TOUCH_DELAY);
	}
}

static void vLcdTask(void *pvParameters)
{
	for(;;)
	{
	GPIO_TogglePinsOutput(GPIO, BOARD_LED2_GPIO_PORT, 1u << BOARD_LED2_GPIO_PIN);
	vTaskDelay(LCD_DELAY);
	}
	
}


/* receive data */
void USART_ReceiveData()
{ 
	uint8_t i_cnt;
	if(USART_RX_First==1&&USART_RX_Last==1)
	{
		for(i_cnt=0;i_cnt<DEMO_RING_BUFFER_SIZE;i_cnt++)
		VfUSART_Data[i_cnt]=demoRingBuffer[i_cnt];
		if(VfUSART_Data[0]==0xFF)
		{
			OBD_Service_Mode_Detection=VfUSART_Data[1];
		}
		rxIndex=0;
		USART_RX_First=0;
		USART_RX_Last=0;
		USART_RX_Wait=0;
		
	}
	else if(USART_RX_First==1&&USART_RX_Last==0)
	{
		if(USART_RX_Wait==0)
		{
		USART_RX_Wait=1;
		}
		else
		{
		rxIndex=0;
		USART_RX_First=0;
		USART_RX_Last=0;
		USART_RX_Wait=0;
		}
		
	}		
	else
	{
		rxIndex=0;
		USART_RX_First=0;
		USART_RX_Last=0;
		USART_RX_Wait=0;
	}
			
}


