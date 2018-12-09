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

bool message_transmitted = false;
		
bool obd_can_TxMSG_Standard(CAN_Type *base, uint8_t mbIdx, can_frame_t *txFrame)
{
	
	        /* time to send messages from CAN0 */
           /* send 0x100 -> 0x102 on tx message buffer 0 */
	
            //for (b = 0; b < txFrame.length; b++) txFrame.dataByte[b] = b;
            /* use message buffer 0 */
            if (CAN_TransferSendBlocking(CAN0, 0, txFrame) != kStatus_Success)
            {
              PRINTF("Failed to transmit message\r\n");
							
            }
            else
            {
                /* toggle LED1 */
                //GPIO_TogglePinsOutput(GPIO, BOARD_LED1_GPIO_PORT, 1u << BOARD_LED1_GPIO_PIN);
              return true;
                //message_transmitted = true;
             }

}
void obd_can_TxMSG_Extend(CAN_Type *base, uint8_t mbIdx, can_frame_t *txFrame)
{
}
