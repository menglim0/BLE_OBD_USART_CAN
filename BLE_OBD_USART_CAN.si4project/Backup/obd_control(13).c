#include "obd_control.h"
#include "board.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "can.h"
#include "fsl_usart.h"
#include "obd_can.h"
#include "obd_usart.h"

#define DebugModeEnable 1

bool G_OBD_Control_Status,G_OBD_Sending_CMD,G_OBD_Receive_CMD,G_OBD_BLE_Disconnect;

uint8_t Usart_Received_Control_Feedback_1[14]={0xA1,0x01,0x00,0x00,0x04,0xC9,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
 uint8_t Usart_Send_BLE_Connected[14]={0x54,0x54,0x4D,0x3A,0x4F,0x4B,0x0D,0x0A,0x00,0x88,0x99,0xaa,0xbb,0xcc};

 uint32_t BLE_Receive_Service_ID_List[8];
uint8_t BLE_Receive_Service_index;

 TeOBD_Control_MODE BLE_Command_Mode;

//#define Datalength 14;

void vControl_Status(uint8_t data[14])
{
uint8_t datalength;


	vControl_Status_Detection(data,DebugModeEnable);

		
		
		switch( BLE_Command_Mode)
	{
			case CeOBD_Control_BLE_Connect: //Call USART sending function to confrim the connect
				
				vControl_BLE_Connect(data,DebugModeEnable);
				//message_updateToTransmit=true;
					
			break;

			case CeOBD_Control_CAN_Reinit: // Call CAN init function to reinit the CAN Config
				

				//message_updateToTransmit=true;
				
			break;

			
			case CeOBD_Control_CAN_Send: // Send the CAN command 
				

			//	message_updateToTransmit=true;
					
			break;

			
			
			case CeOBD_Control_CAN_Receive: //Receive the CAN data and send by USART
				

			//	message_updateToTransmit=true;
					
			break;
  
			
			default:
				

				//message_updateToTransmit=true;
					
			break;

		}
}


void vControl_CAN_Reinit(uint8_t data[14],bool debugmode)
{
}

void vControl_CAN_Send(uint8_t data[14],bool debugmode)
{
}

void vControl_CAN_Receive(uint8_t data[14],bool debugmode)
{

}

void vControl_BLE_Connect(uint8_t data[14],bool debugmode)
{
	if(0)
	{
		PRINTF("\r\n BLE_Connect \n");
	}
	USART_WriteBlocking(USART0,Usart_Send_BLE_Connected,14);
	BLE_Command_Mode=CeOBD_Control_null;
}

void vControl_Status_Detection(uint8_t data[14],bool debugmode)
{

	/* 判断连接状态，每次上电判断一次，直到连接中断再判断一次*/
 if(G_OBD_Control_Status==false)
{

	G_OBD_Control_Status= usart_Confirm_BLE_Connected(data,8);
	if(G_OBD_Control_Status == true)
	{
			BLE_Command_Mode =CeOBD_Control_BLE_Connect;
	}
	
}
 else
 {
	BLE_Command_Mode =CeOBD_Control_null;

 }

 //if()

 if((data[0]&0x0F) ==3)
 {
	G_OBD_Receive_CMD =true;
	 
	 BLE_Receive_Service_index = data[1];
	 BLE_Receive_Service_ID_List[BLE_Receive_Service_index]= ((uint32_t)data[2]<<24)+((uint32_t)data[3]<<16)+((uint32_t)data[4]<<8)+data[5];
	 if(debugmode)
	{
		PRINTF("\r\n Service_ID + %d \n",BLE_Receive_Service_ID_List[BLE_Receive_Service_index]);
	}
 }
}



