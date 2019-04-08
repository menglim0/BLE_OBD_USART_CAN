#ifndef _OBD_CONTROL_H_
#define _OBD_CONTROL_H_

#include "board.h"
#include "fsl_common.h"


 /*
 初始状态是null
 第一次收到应该是BLE连接信息--> 返回信息后返回到Null
 默认上电后初始化500K，无 CANFD--> 记录Old状态，初始化完成后到Null，需要重新配置发送和接收
 执行发送指令，如果执行完成后返回Null 或者Receive(如果是从Receive状态切换过来)
 Receive状态下可以执行Sending指令 
 */

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
void vControl_CAN_Reinit(uint8_t data[14],bool debugmode);
void vControl_CAN_Send(uint8_t data[14],bool debugmode);
void vControl_CAN_Receive(uint8_t data[14],bool debugmode);
void vControl_BLE_Connect(uint8_t data[14],bool debugmode);

void vControl_Status_Detection(uint8_t data[14],bool debugmode);


#endif /* _OBD_USART_H_ */
