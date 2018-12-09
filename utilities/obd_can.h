/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *

 */
 
#ifndef _OBD_CAN_H_
#define _OBD_CAN_H_
#include "can.h"

bool obd_can_TxMSG_Standard(CAN_Type *base, uint8_t mbIdx, can_frame_t *txFrame);
void obd_can_TxMSG_Extend(CAN_Type *base, uint8_t mbIdx, can_frame_t *txFrame);


#endif /* _OBD_CAN_H_ */






