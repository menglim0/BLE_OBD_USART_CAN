/*
 * Create 2018/12/08
 Function: for CAN obd service;
 Receive from the BLE module via the USART
 Transmint the data to the P-CAN with UDS service;
 Receive the specifical signals from CAN ;
  
 */


#include "obd_usart.h"
#include "board.h"

uint8_t BLE_Connect[]={0x54,0x54,0x4D,0x3A,0x4F,0x4B,0x0D,0x0A,0x00};
bool USART_RX_First=0,USART_RX_Last=0,USART_RX_Wait=0;       //接收状态标记




void usart_Data_convert(uint8_t databuffer[14])
{
	
	
	
	
}

bool usart_Confirm_BLE_Connected(uint8_t *receiveddata,uint8_t length)
{
	
	uint8_t i;
	for(i=0;i<length;i++)
		{
			if(receiveddata[i]!=BLE_Connect[i])
			{
				return false;
			}
			
		}

	return true;
}

/* check if the data log fully*/
void USART_ReceiveData(uint8_t *receiveddata,uint8_t length,uint8_t *result)
{ 
	uint8_t i_cnt;
	if(USART_RX_First==1&&USART_RX_Last==1)
	{
		for(i_cnt=0;i_cnt<length;i_cnt++)
		result[i_cnt]=receiveddata[i_cnt];
		if(result[0]==0xFF)
		{
			//OBD_Service_Mode_Detection=VfUSART_Data[1];
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





