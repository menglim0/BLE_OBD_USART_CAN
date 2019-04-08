#ifndef _OBD_CONTROL_H_
#define _OBD_CONTROL_H_

#include "board.h"
#include "fsl_common.h"

 typedef enum 
 {
	 CeOBD_Control_null,				/* 00 */
	 CeOBD_Control_BLE_Connect, 		/* 01 */
	CeOBD_Control_CAN_Reinit,			/* 02 */	  
	CeOBD_Control_CAN_Send, 		   /* 03 */
	CeOBD_Control_CAN_Receive,			   /* 04 */
	CeOBD_Control_Fault_Connect,		   /* 05 mutil frame*/		
	
 } TeOBD_Control_MODE;
 





void  vControl_Status(uint8_t data[14]);
void vControl_CAN_Reinit(uint8_t data[14]);
void vControl_CAN_Send(uint8_t data[14]);
void vControl_CAN_Receive(uint8_t data[14]);
void vControl_BLE_Connect(uint8_t data[14]);



#endif /* _OBD_USART_H_ */
