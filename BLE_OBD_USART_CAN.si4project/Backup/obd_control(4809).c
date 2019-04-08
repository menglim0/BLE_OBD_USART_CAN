#include "obd_control.h"
#include "board.h"

bool G_OBD_Control_Status;



 void vControl_Status(uint8_t data[14])
	{
	 if(G_OBD_Control_Status==false)
			{

				G_OBD_Control_Status= usart_Confirm_BLE_Connected(data,8);
			//	BLE_Command_Mode =CeOBD_Control_BLE_Connect;
		//		usart_Receive_Complete=false;
		//		USART_rxIndex=0;
			
			}
	}
