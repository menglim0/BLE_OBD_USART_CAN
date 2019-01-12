/*
 * Create 2018/12/08
 Function: for CAN obd service;
 Receive from the BLE module via the USART
 Transmint the data to the P-CAN with UDS service;
 Receive the specifical signals from CAN ;
  
 */


#include "obd_can.h"
#include "can.h"
#include <stdbool.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "can.h"

/*The Service list*/
uint8_t Service_KeepAlive[8] =  {0x02,0x3E,0x80,0x00,0x00,0x00,0x00,0x00};
uint8_t Service_ExtSession[8]=  {0x02,0x10,0x03,0x00,0x00,0x00,0x00,0x00};
uint8_t Service_ReqSeed[8]   =  {0x02,0x27,0x09,0x00,0x00,0x00,0x00,0x00};

uint8_t Service_SendKey[3][8]={ {0x10,0x0E,0x27,0x0A,0x20,0x20,0x20,0x20},
																{0x21,0x20,0x20,0x20,0x20,0x20,0x20,0x20},
																{0x22,0x20,0x00,0x00,0x00,0x00,0x00,0x00}};

																

bool message_transmitted = false;
bool message_updateToTransmit = false;
uint8_t message_length = 8;
uint8_t message_length_cnt;
can_frame_t FrameToTransmit;	
																

																
TeOBD_Service_MODE OBD_Service_Mode_CMD;	
TeOBD_Service_MODE OBD_Service_Mode_NextState;	
																
																
bool obd_Service(TeOBD_Service_MODE OBD_Service_Mode_CMD)
{
		FrameToTransmit.id=0x14dae1f1;
		FrameToTransmit.format =kCAN_FrameFormatExtend;
		FrameToTransmit.type = kCAN_FrameTypeData;
		FrameToTransmit.proto = kCAN_ProtoTypeClassic;
		FrameToTransmit.bitratemode = kCAN_BitrateModeTypeSwitch;
		FrameToTransmit.length = 8;
	
	if(OBD_Service_Mode_CMD!=CeOBD_Service_MODE_3E_KeepAlive)
	{
	OBD_Service_Mode_CMD=OBD_Service_Mode_NextState;
	}
	
	switch( OBD_Service_Mode_CMD)
	{
			case CeOBD_Service_MODE_3E_KeepAlive:
				
				for(message_length_cnt=0;message_length_cnt<message_length;message_length_cnt++)
				{
					FrameToTransmit.dataByte[message_length_cnt]=Service_KeepAlive[message_length_cnt];
				}
				message_updateToTransmit=true;
				OBD_Service_Mode_NextState=	CeOBD_Service_MODE_10_ExtSession;		
			break;
			
			case CeOBD_Service_MODE_10_ExtSession:
				for(message_length_cnt=0;message_length_cnt<message_length;message_length_cnt++)
				{
					FrameToTransmit.dataByte[message_length_cnt]=Service_ExtSession[message_length_cnt];
				}
				message_updateToTransmit=true;
				OBD_Service_Mode_NextState=	CeOBD_Service_MODE_29_ReqSeed;						
			break;
				
			case CeOBD_Service_MODE_29_ReqSeed:
				for(message_length_cnt=0;message_length_cnt<message_length;message_length_cnt++)
				{
					FrameToTransmit.dataByte[message_length_cnt]=Service_ReqSeed[message_length_cnt];
				}
				message_updateToTransmit=true;	

				OBD_Service_Mode_NextState=	CeOBD_Service_MODE_0E_SendKey_MF;			
				
			break;
					
			case CeOBD_Service_MODE_0E_SendKey_MF:
				for(message_length_cnt=0;message_length_cnt<message_length;message_length_cnt++)
				{
					FrameToTransmit.dataByte[message_length_cnt]=Service_SendKey[0][message_length_cnt];
				}
				message_updateToTransmit=true;		
				
								OBD_Service_Mode_NextState=	CeOBD_Service_MODE_0E_SendKey_MF1;		
			break;
				
				case CeOBD_Service_MODE_0E_SendKey_MF1:
				for(message_length_cnt=0;message_length_cnt<message_length;message_length_cnt++)
				{
					FrameToTransmit.dataByte[message_length_cnt]=Service_SendKey[1][message_length_cnt];
				}
				message_updateToTransmit=true;		
				
								OBD_Service_Mode_NextState=	CeOBD_Service_MODE_0E_SendKey_MF2;		
			break;
				
			case CeOBD_Service_MODE_0E_SendKey_MF2:
				for(message_length_cnt=0;message_length_cnt<message_length;message_length_cnt++)
				{
					FrameToTransmit.dataByte[message_length_cnt]=Service_SendKey[2][message_length_cnt];
				}
				message_updateToTransmit=true;		
				
				OBD_Service_Mode_NextState=	CeOBD_Service_MODE_No_update;		
			break;
						
			case CeOBD_Service_MODE_No_update:

			message_updateToTransmit=false;
			break;
							
			default:
				message_updateToTransmit=false;
		
			break;

	}
	
	if(message_updateToTransmit)
	{
		//obd_Service_MsgTrasmit(&FrameToTransmit);
		CAN_TransferSendBlocking(CAN0, 0, &FrameToTransmit);
	}
	
return 0;
}	


void obd_Service_MsgTrasmit(can_frame_t *txFrame)
{
	
	obd_can_TxMSG_Standard(CAN0, 0, txFrame);
	
}

		
bool obd_can_TxMSG_Standard(CAN_Type *base, uint8_t mbIdx, can_frame_t *txFrame)
{
	
	        /* time to send messages from CAN0 */
           /* send 0x100 -> 0x102 on tx message buffer 0 */
	
            //for (b = 0; b < txFrame.length; b++) txFrame.dataByte[b] = b;
            /* use message buffer 0 */
            if (CAN_TransferSendBlocking(CAN0, mbIdx, txFrame) != kStatus_Success)
            {
              PRINTF("transmit");
							
            }
            else
            {
                /* toggle LED1 */
                //GPIO_TogglePinsOutput(GPIO, BOARD_LED1_GPIO_PORT, 1u << BOARD_LED1_GPIO_PIN);
              return true;
                //message_transmitted = true;
             }
		return true;
}

void obd_can_TxMSG_Extend(CAN_Type *base, uint8_t mbIdx, can_frame_t *txFrame)
{
}
