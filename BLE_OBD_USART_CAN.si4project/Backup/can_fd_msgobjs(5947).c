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
#include "obd_usart.h"

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
#define LCD_DELAY   (20)


static void vTouchTask(void *pvParameters);
static void vLcdTask(void *pvParameters);

//void USART_ReceiveData(void);

TaskHandle_t xTouchTaskHandle = NULL;
TaskHandle_t xLcdTaskHandle = NULL;
 
volatile uint32_t g_systickCounter;



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

uint16_t rxIndex_old,rxIndex_count,rxIndex_loop,delay_count,debug_count;
bool rxIndex_updated,tx_CAN_Enable,message_received,Keep_Service_Active,KeepAlive_PERIOD_flag;
bool Keep_Service_Active_Send;
uint16_t Rx_Msg_Cnt,Rx_Msg_Loop_Cnt;

#define KeepAlive_Peroid_Cnt_2s (2000/TOUCH_DELAY)


uint16_t KeepAlive_Peroid_2s_Count;


uint8_t VfCANH_RxMSG_Data;
uint16_t VfCANH_RxMSG_ID,Array_Cycle,USART_rxIndex,KeepSendTimeCnt,KeepSendOneTime;

uint8_t VfUSART_Data[DEMO_RING_BUFFER_SIZE];
uint8_t USART_Data[DEMO_RING_BUFFER_SIZE],i;
uint8_t demoRingBuffer[DEMO_RING_BUFFER_SIZE];
uint8_t demoRingBuffer_init[DEMO_RING_BUFFER_SIZE]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint8_t Usart_Send_Test[14]={0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee};

uint8_t Usart_Received_Feedback_1[14]={0xA1,0x01,0x00,0x00,0x04,0xC9,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint8_t Usart_Received_Feedback_2[14]={0xA1,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};




uint8_t ReceiveDataFromCAN_to_USART[12];

usart_handle_t usart0_Define;
bool usart_first_Datareceived,usart_Receive_Complete;

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


bool G_Ble_Connect_Status;






typedef enum
{
	CeOBD_Receive_list_0,                /* 00 */
   CeOBD_Receive_list_1,           /* 01 */      
   CeOBD_Receive_list_2,            /* 02 */
   CeOBD_Receive_list_3,             /* 03 */
   CeOBD_Receive_list_4,           /* 04 mutil frame*/      
   
} TeOBD_Receive_list;

TeOBD_Receive_list BLE_Command_Receive_list;


																
can_frame_t Rxmsg_TransOilTem = { 0 };

void vBLE_Command_Mode_Action(TeOBD_Control_MODE cmdMode);

/*******************************************************************************
 * Code
 ******************************************************************************/
 void DEMO_USART_IRQHandler(void)
{
    uint8_t data,data_length;
		rxIndex_updated=false;
		delay_count=0;

	
    /* If new data arrived. */
    if ((kUSART_RxFifoNotEmptyFlag | kUSART_RxError) & USART_GetStatusFlags(DEMO_USART))
    {
			if(usart_Receive_Complete==false)
			{
      			data = USART_ReadByte(DEMO_USART);

      			demoRingBuffer[USART_rxIndex] = data;
        		USART_rxIndex++;
			}
			else
			{
				data = USART_ReadByte(DEMO_USART);
			}
			
			if(usart_first_Datareceived ==false)
			{
				
				data_length = 9;				
			}
			else
			{
				data_length=demoRingBuffer[0]>>4;	
			}

			if(data_length ==USART_rxIndex  )
			{	
				usart_first_Datareceived=true;				
				usart_Receive_Complete=true;
				//USART_rxIndex=0;
			}

				  
    }
		
	
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
 
    /* Board pin, clock, debug console init */
			
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);
    CLOCK_EnableClock(kCLOCK_Gpio2);
    CLOCK_EnableClock(kCLOCK_Gpio3);

	/* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
		
    BOARD_InitPins();
		//BOARD_BootClockFROHF48M();
	BOARD_BootClockPLL180M();
  BOARD_InitDebugConsole();
	BOARD_InitCAN();
	BOARD_InitGPIO();

    /* print a note to terminal */
    PRINTF("\r\n CAN-FD \n");
   
    /* Enable SysTick Timer */
    SysTick_Config(SystemCoreClock / TICKRATE_HZ);

    USART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;

    USART_Init(DEMO_USART, &config, DEMO_USART_CLK_FREQ);
    USART_EnableInterrupts(DEMO_USART, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable);
    EnableIRQ(DEMO_USART_IRQn);
	
 		
	xTaskCreate(vTouchTask,"Touch Task",TOUCHTASK_STACKSIZE,NULL,TOUCHTASK_PRIORITY,&xTouchTaskHandle);
	xTaskCreate(vLcdTask,"LCD Task",LCDTASK_STACKSIZE,NULL,LCDTASK_PRIORITY,&xLcdTaskHandle);
	vTaskStartScheduler();
		
	while(1)
	{
		;
	}
	

}

static void vTouchTask(void *pvParameters)
{
	
	
	 bool BLE_Connect_Status;
	uint8_t i;
	//创建消息队列
    //Key_Queue=xQueueCreate(KEYMSG_Q_NUM,sizeof(uint8_t));        //创建消息Key_Queue
    //Message_Queue=xQueueCreate(MESSAGE_Q_NUM,USART_REC_LEN); //创建消息Message_Queue,队列项长度是串口接收缓冲区长度
	for(;;)
	{

		/*confirm the logic running*/
			KeepSendOneTime=KeepSendTimeCnt%10;
		
		if(KeepSendOneTime==0)
		{
		
			GPIO_TogglePinsOutput(GPIO, BOARD_LED3_GPIO_PORT, 1u << BOARD_LED3_GPIO_PIN);
		
		}
		
			if(	usart_first_Datareceived==true&&usart_Receive_Complete==true)
			{
			    vControl_Status(demoRingBuffer);
				usart_first_Datareceived=false;				
				usart_Receive_Complete=false;
				USART_rxIndex=0;
				for(i=0;i<14;i++)
				{

				demoRingBuffer[i]=0;
				}
			}

						KeepSendTimeCnt++;
			
			//USART_ReceiveData()
				/*    if(G_Ble_Connect_Status==false)
					{

						G_Ble_Connect_Status= usart_Confirm_BLE_Connected(demoRingBuffer,8);
						BLE_Command_Mode =CeOBD_Control_BLE_Connect;
						usart_Receive_Complete=false;
						USART_rxIndex=0;
					
					}
					else
					{

							//xQueueReceive(Message_Queue,&key,portMAX_DELAY);
						if(usart_Receive_Complete==true)
						{
							//USART_ReceiveData(demoRingBuffer,USART_rxIndex,VfUSART_Data);
							
							uint8_t i_cnt;

							for(i_cnt=0;i_cnt<USART_rxIndex;i_cnt++)
							{
								VfUSART_Data[i_cnt]=demoRingBuffer[i_cnt];
							}

							if( (VfUSART_Data[0]&0x0F) ==3 )
								{
								
								}
							USART_rxIndex=0;
							usart_Receive_Complete=false;
						}
							if(VfUSART_Data[0]==0xFF&&VfUSART_Data[1] ==0x01)
							{
								//OBD_Service_Mode_Detection=1;
								VfUSART_Data[0]=0x00;
								VfUSART_Data[1]=0x00;
								Keep_Service_Active=true;
								KeepAlive_Peroid_2s_Count=0;
							}
							obd_Service(1);
							KeepSendOneTime++;
							
							
						//	OBD_Service_Mode_Detection=0;
							
							if(GPIO_ReadPinInput(GPIO,BOARD_SW5_GPIO_PORT,BOARD_SW5_GPIO_PIN)==false)
							{
								GPIO_TogglePinsOutput(GPIO, BOARD_LED1_GPIO_PORT, 1u << BOARD_LED1_GPIO_PIN);
								
								for (rxIndex_loop=0;rxIndex_loop<14;rxIndex_loop++)
								{						
										//USART_WriteByte(DEMO_USART, Usart_Send_Test[rxIndex_loop]);					
								}

								USART_WriteBlocking(DEMO_USART,Usart_Send_Test,14);
						
					}*/

	//}
		
		
		
	//	if(Keep_Service_Active==true)
	if(1)
		{
			KeepAlive_Peroid_2s_Count++;
			if(KeepAlive_Peroid_2s_Count>=KeepAlive_Peroid_Cnt_2s)
			{
				obd_Service_KeepAlive();
				KeepAlive_Peroid_2s_Count=0;
			}
	
		}
		
		vTaskDelay(TOUCH_DELAY);
	}
}

static void vLcdTask(void *pvParameters)
{
	for(;;)
	{
	Rx_Msg_Loop_Cnt++;
		
		Rxmsg_TransOilTem.id=0;
	/*mask with 0x4C9*/
	if (CAN_ReadRxMb(CAN0, 0, &Rxmsg_TransOilTem) == kStatus_Success)
	{


		for(i=0;i<8;i++)
		{
			Usart_Received_Feedback_1[6+i]=Rxmsg_TransOilTem.dataByte[i];
		}

		USART_WriteBlocking(DEMO_USART,Usart_Received_Feedback_1,14);
		Rx_Msg_Cnt++;
		
	}

	
	//vBLE_Command_Mode_Action(BLE_Command_Mode);

	
	vTaskDelay(LCD_DELAY);
	
	}
	
}








