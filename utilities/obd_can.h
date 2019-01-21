/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *

 */
 
#ifndef _OBD_CAN_H_
#define _OBD_CAN_H_
#include "can.h"

/* OBD-II Modes */
#define OBD_MODE_SHOW_CURRENT_DATA        0x01
#define OBD_MODE_SHOW_FREEZE_FRAME        0x02
#define OBD_MODE_READ_DTC                 0x03
#define OBD_MODE_CLEAR_DTC                0x04
#define OBD_MODE_TEST_RESULTS_NON_CAN     0x05
#define OBD_MODE_TEST_RESULTS_CAN         0x06
#define OBD_MODE_READ_PENDING_DTC         0x07
#define OBD_MODE_CONTROL_OPERATIONS       0x08
#define OBD_MODE_VEHICLE_INFORMATION      0x09
#define OBD_MODE_READ_PERM_DTC            0x0A

typedef enum
{
	CeOBD_Service_MODE_Inactive,                /* 00 */
   CeOBD_Service_MODE_3E_KeepAlive,           /* 01 */      
   CeOBD_Service_MODE_10_ExtSession,            /* 02 */
   CeOBD_Service_MODE_29_ReqSeed,             /* 03 */
   CeOBD_Service_MODE_0E_SendKey_MF,           /* 04 mutil frame*/      
	   CeOBD_Service_MODE_0E_SendKey_MF1,        /* 05 mutil frame1*/
	   CeOBD_Service_MODE_0E_SendKey_MF2, /* 03 mutil frame2*/
	 CeOBD_Service_MODE_No_update         /* No_Update*/
} TeOBD_Service_MODE;

extern TeOBD_Service_MODE OBD_Service_Mode_CMD;	

bool obd_can_TxMSG_Standard(CAN_Type *base, uint8_t mbIdx, can_frame_t *txFrame);

bool obd_Service(TeOBD_Service_MODE state);

void obd_Service_MsgTrasmit(can_frame_t *txFrame);

void obd_can_TxMSG_Extend(CAN_Type *base, uint8_t mbIdx, can_frame_t *txFrame);

bool obd_Service_KeepAlive();



#endif /* _OBD_CAN_H_ */






