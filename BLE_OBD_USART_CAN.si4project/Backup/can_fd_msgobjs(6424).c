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
#include "obd_control.h"
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
#define TOUCH_DELAY   (50)
#define LCD_DELAY   (20)


static void vTouchTask(void *pvParameters);
static void vLcdTask(void *pvParameters);

void vTask_UsartReceive_Detection();
void vTask_UsartReceive_UnPack();


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


uint16_t KeepAlive_Peroid_2s_Count,total_index;


uint8_t VfCANH_RxMSG_Data;
uint16_t VfCANH_RxMSG_ID,Array_Cycle,USART_rxIndex,KeepSendTimeCnt,KeepSendOneTime;

uint8_t VfUSART_Data[DEMO_RING_BUFFER_SIZE];
uint8_t USART_Data[DEMO_RING_BUFFER_SIZE],i;
uint8_t demoRingBuffer[DEMO_RING_BUFFER_SIZE];
uint8_t demoRingBuffer_Total[280];
uint8_t demoRingBuffer_init[DEMO_RING_BUFFER_SIZE]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint8_t Usart_Send_Test[14]={0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee};

uint8_t Usart_Received_Feedback_1[14]={0xA1,0x81,0x00,0x00,0x04,0xC9,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint8_t Usart_Received_Feedback_2[14]={0xA1,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint8_t Multiframe_FireWall_Cmd[10][14];

uint8_t Usart_Config_Init[14];

uint8_t Multiframe_FireWall_index,Multiframe_Control_index,FrameIndex,FrameIndex_rest;
bool Multiframe_FireWall_Send;
/**
Multiframe_FireWall_index=1 -->Multiframe_FireWall_Cmd
*/

uint8_t ReceiveDataFromCAN_to_USART[12];

usart_handle_t usart0_Define;
bool usart_first_Datareceived,usart_Receive_Complete;

uint8_t g_tipString[] =
    "Usart functional API interrupt example\r\nBoard receives characters then sends them out\r\nNow please input:\r\n";
		
uint8_t Multi_Frame_Key[2][8]={{0x21,0x20,0x20,0x20,0x20,0x20,0x20,0x20},
																{0x22,0x20,0x00,0x00,0x00,0x00,0x00,0x00}};

																
//������Ϣ���е�����
#define KEYMSG_Q_NUM    1  		//������Ϣ���е�����  
#define MESSAGE_Q_NUM   4   	//�������ݵ���Ϣ���е����� 
QueueHandle_t Key_Queue;   		//����ֵ��Ϣ���о��
QueueHandle_t Message_Queue;	//��Ϣ���о��
#define USART_REC_LEN  			20  	//�����������ֽ��� 50
																
/* Usart queue*/
extern QueueHandle_t Message_Queue;	//��Ϣ���о��


bool G_Ble_Connect_Status;






typedef enum
{
	CeOBD_Receive_list_0,                /* 00 */
   CeOBD_Receive_list_1,           /* 01 */      
   CeOBD_Receive_list_2,            /* 02 */
   CeOBD_Receive_list_3,             /* 03 */
   CeOBD_Receive_list_4,           /* 04 mutil frame*/      
   
} TeOBD_Receive_list;


typedef enum
{
	CeOBD_Receive_NoCmd,                /* 00 */
   CeOBD_Receive_Config_init,           /* 01 */      
   CeOBD_Receive_Sending_Cmd,            /* 02 */
   CeOBD_Receive_Sending_Cmd_Multiframe,             /* 03 */
   CeOBD_Receive_Complete,           /* 04 mutil frame*/      
   
} TeOBD_Receive_Cmd;

typedef enum
{
	 CeUSART_Receive_inactive,                /* 00 */
   CeUSART_Receive_Start,           /* 01 */      
   CeUSART_Receive_OnGoing,            /* 02 */
	CeUSART_Receive_Complete,             /* 03 */               
   
} TeUSART_Receive_State;

TeOBD_Receive_list BLE_Command_Receive_list;
TeOBD_Receive_Cmd BLE_Receive_Command;


TeUSART_Receive_State  VeUSART_Receive_State;
																
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
			
      			data = USART_ReadByte(DEMO_USART);
      			demoRingBuffer[USART_rxIndex] = data;
				demoRingBuffer_Total[total_index]=data;
				total_index++;
				VeUSART_Receive_State= CeUSART_Receive_Start;

				
			if(total_index>=280)
			{
			total_index=0;
			}
				/*All cmd start as 0xE...*/
			
				if((demoRingBuffer[0]>>4)==0x0E)
				{
					usart_first_Datareceived=true;//Start Valid frame
				}
				
			if(usart_first_Datareceived==false)
			{
        		USART_rxIndex=0;
			}
			else
			{
				USART_rxIndex++;
			}
						
			if(USART_rxIndex == (demoRingBuffer[0]>>4)  )
			{	
								
				usart_Receive_Complete=true;
				USART_rxIndex=0;
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
	can_frame_t tx_frame1;
	//������Ϣ����
    //Key_Queue=xQueueCreate(KEYMSG_Q_NUM,sizeof(uint8_t));        //������ϢKey_Queue
    //Message_Queue=xQueueCreate(MESSAGE_Q_NUM,USART_REC_LEN); //������ϢMessage_Queue,��������Ǵ��ڽ��ջ���������
	for(;;)
	{

		/*confirm the logic running*/
			KeepSendOneTime=KeepSendTimeCnt%10;
		
		if(KeepSendOneTime==0)
		{
		
			GPIO_TogglePinsOutput(GPIO, BOARD_LED3_GPIO_PORT, 1u << BOARD_LED3_GPIO_PIN);
		
		}
	
		KeepSendTimeCnt++;


		vTask_UsartReceive_Detection();
	
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
	uint8_t ReceiveIndex,ReceiveIndex_mask=0;
	can_frame_t tx_frame1,Rx_frame_Mask_temp;
	//can_frame_t Rx_frame_Mask[3]={0,0,0};
	uint32_t Rx_frame_ID[3];
	bool Rx_ID_Enabled[3]={false,false,false};
	
	
	for(;;)
	{
		
		//Rx_Msg_Loop_Cnt++;
		
		/*
		if(usart_first_Datareceived==true&&Rx_Msg_Loop_Cnt<=1)
		{
			Rx_Msg_Loop_Cnt++;
		}
		else
		{
			Rx_Msg_Loop_Cnt=0;
			USART_rxIndex=0;
			for(i=0;i<14;i++)
				{

				demoRingBuffer[i]=0;
				}
		}
			
		if(usart_first_Datareceived==false&&usart_Receive_Complete==true)
		{
			usart_first_Datareceived=false;				
			usart_Receive_Complete=false;
			USART_rxIndex=0;
			for(i=0;i<14;i++)
				{
				Multiframe_FireWall_Cmd[demoRingBuffer[14]][i]=demoRingBuffer[i];
				demoRingBuffer[i]=0;
				}
		}
		
		
			if(	usart_first_Datareceived==true&&usart_Receive_Complete==true)
			{
				GPIO_TogglePinsOutput(GPIO, BOARD_LED2_GPIO_PORT, 1u << BOARD_LED2_GPIO_PIN);
			  vControl_Status(demoRingBuffer);
				usart_first_Datareceived=false;				
				usart_Receive_Complete=false;
				USART_rxIndex=0;
				if(demoRingBuffer[0]==0xE1)
				{
					if(demoRingBuffer[13]==0)
					{
						tx_frame1=obd_can_TxMSG_Pack(demoRingBuffer);	
										
						if(tx_frame1.id!=0)
						{
							obd_Service_MsgTrasmit(	&tx_frame1);
							KeepAlive_Peroid_2s_Count=0;
						}
					}
					else
					{
						Multiframe_FireWall_index=1;
						Multiframe_FireWall_Send=true;
					}
				}
				
				
				
				if(demoRingBuffer[0]==0xE3)
				{
				for(ReceiveIndex_mask=0;ReceiveIndex_mask<3;ReceiveIndex_mask++)
					{								
						if((demoRingBuffer[5]&0x0F)==ReceiveIndex_mask+1)
						{													
							Rx_frame_ID[ReceiveIndex_mask]=obd_can_RxMSG_UnPack(demoRingBuffer);
							//Rx_frame_ID[ReceiveIndex_mask] = Rx_frame_Mask_temp.id;
							
							//CAN_SetRxIndividualMask(CAN0, ReceiveIndex_mask, CAN_RX_MB_STD(Rx_frame_ID[ReceiveIndex_mask], 0));						
							
							if((demoRingBuffer[5]&0x80)==0x80)
							{
								Rx_ID_Enabled[0]=false;
							}
							else
							{
								Rx_ID_Enabled[0]=true;
							}
						}
					}
						
				}
				
				for(i=0;i<14;i++)
				{  if(demoRingBuffer[13]!=0)
					{
						Multiframe_FireWall_Cmd[demoRingBuffer[13]][i]=demoRingBuffer[i];
					}
				demoRingBuffer[i]=0;
				}
			}
			
			*/

	if(BLE_Receive_Command == CeOBD_Receive_Sending_Cmd_Multiframe)
	{
		tx_frame1=obd_can_TxMSG_Pack(Multiframe_FireWall_Cmd[Multiframe_Control_index]);	
												
		if(tx_frame1.id!=0)
		{
			obd_Service_MsgTrasmit( &tx_frame1);
			KeepAlive_Peroid_2s_Count=0;
		}

		if(Multiframe_Control_index>=FrameIndex)
		{
			BLE_Receive_Command=CeOBD_Receive_NoCmd;
			Multiframe_Control_index=0;
		}
		else
		{
			Multiframe_Control_index++;
		}
	}
		
			
			if(0)
			{
				if(Multiframe_FireWall_index==1)
				{
					Multiframe_Control_index++;
					
					if(Multiframe_FireWall_Cmd[Multiframe_Control_index][13]==Multiframe_Control_index)
					{
						tx_frame1=obd_can_TxMSG_Pack(Multiframe_FireWall_Cmd[Multiframe_Control_index]);	
										
						if(tx_frame1.id!=0)
						{
							obd_Service_MsgTrasmit(	&tx_frame1);
							KeepAlive_Peroid_2s_Count=0;
						}
					}
					
					if(Multiframe_Control_index>=6)
					{
					Multiframe_FireWall_Send=false;
					Multiframe_Control_index=0;
					}
				}
			}
			else
			{
				Multiframe_Control_index=0;
			}
				
	/*mask with 0x4C9*/
		//for(ReceiveIndex=0;ReceiveIndex<4;ReceiveIndex++)
		//{
			//Rxmsg_TransOilTem.id=0x4C9;
			
			//Rxmsg_TransOilTem.id = BLE_Receive_Service_ID_List[ReceiveIndex+1];
			
			if (CAN_ReadRxMb(CAN0,0, &Rx_frame_Mask_temp) == kStatus_Success)
				{
					for(i=0;i<8;i++)
					{
						Usart_Received_Feedback_1[6+i]=Rx_frame_Mask_temp.dataByte[i];
					}
					
					Usart_Received_Feedback_1[2]=ReceiveID_Setting[0];
					Usart_Received_Feedback_1[3]=ReceiveID_Setting[1];
					Usart_Received_Feedback_1[4]=Rx_frame_Mask_temp.id>>8;
					Usart_Received_Feedback_1[5]=Rx_frame_Mask_temp.id&0xFF;
					
						USART_WriteBlocking(DEMO_USART,Usart_Received_Feedback_1,14);
					}
				
			/*
			for(ReceiveIndex_mask=0;ReceiveIndex_mask<3;ReceiveIndex_mask++)
			{
			if(Rx_ID_Enabled[ReceiveIndex_mask]==true)
			{
				if (CAN_ReadRxMb(CAN0,ReceiveIndex_mask, &Rx_frame_Mask_temp) == kStatus_Success)
				{
					for(i=0;i<8;i++)
					{
						Usart_Received_Feedback_1[6+i]=Rx_frame_Mask_temp.dataByte[i];
					}
					
					Usart_Received_Feedback_1[2]=ReceiveID_Setting[0];
					Usart_Received_Feedback_1[3]=ReceiveID_Setting[1];
					Usart_Received_Feedback_1[4]=Rx_frame_Mask_temp.id>>8;
					Usart_Received_Feedback_1[5]=Rx_frame_Mask_temp.id&0xFF;
					
						USART_WriteBlocking(DEMO_USART,Usart_Received_Feedback_1,14);
					}
				}
			}*/
					
			//					}
				
			//}

	
	//vBLE_Command_Mode_Action(BLE_Command_Mode);

	
	vTaskDelay(LCD_DELAY);
	
	}
	
}


void vTask_UsartReceive_Detection()
{


	if(usart_first_Datareceived==true&&Rx_Msg_Loop_Cnt<=2)
	{
		Rx_Msg_Loop_Cnt++;
	}
	else
	{
		/*Receive complete, record the data to array*/
		vTask_UsartReceive_UnPack();

		for(i=0;i<total_index;i++)
		{
		demoRingBuffer_Total[i]=0;
		}
		
		Rx_Msg_Loop_Cnt=0;
		USART_rxIndex=0;
		total_index=0;
	}

	
if(VeUSART_Receive_State== CeUSART_Receive_Start)
{

}



}


void vTask_UsartReceive_UnPack()
{

	/*0xEA config the init data*/
	if(demoRingBuffer_Total[0]==0xEA)
	{
		for(i=0;i<14;i++)
		{
			Usart_Config_Init[i]=demoRingBuffer_Total[i];
		}
			VeUSART_Receive_State=CeUSART_Receive_Complete;
		BLE_Receive_Command = CeOBD_Receive_Config_init;
	}

	/*0xE6 config the multiframe data*/
	if(demoRingBuffer_Total[0]==0xE6)
	{
		for(i=0;i<total_index;i++)
		{
			FrameIndex=i/14; 
			FrameIndex_rest = i%14;
			Multiframe_FireWall_Cmd[FrameIndex][FrameIndex_rest]=demoRingBuffer_Total[i];	
		}
		VeUSART_Receive_State=CeUSART_Receive_Complete;
		BLE_Receive_Command = CeOBD_Receive_Sending_Cmd_Multiframe;
	}

}

