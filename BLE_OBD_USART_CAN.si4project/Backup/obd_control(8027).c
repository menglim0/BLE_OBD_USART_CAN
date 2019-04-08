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

bool G_OBD_Control_Status;

uint8_t Usart_Received_Control_Feedback_1[14]={0xA1,0x01,0x00,0x00,0x04,0xC9,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
 uint8_t Usart_Send_BLE_Connected[14]={0x54,0x54,0x4D,0x3A,0x4F,0x4B,0x0D,0x0A,0x00,0x88,0x99,0xaa,0xbb,0xcc};


 TeOBD_Control_MODE BLE_Command_Mode;

//#define Datalength 14;

void vControl_Status(uint8_t data[14])
{
uint8_t datalength;



		
		
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
	if(debugmode)
	{
	
	}
	USART_WriteBlocking(USART0,Usart_Send_BLE_Connected,14);
	BLE_Command_Mode=CeOBD_Control_null;
}

void vControl_Status_Detection(uint8_t data[14])
{

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

}



