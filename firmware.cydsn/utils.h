// -----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// -----------------------------------------------------------------------------

/**
* \file 		utils.h
*
* \brief 		Declaration of utility functions.
* \date 		Feb 16, 2014
* \author 		qbrobotics
* \copyright	(C)  qbrobotics. All rights reserved.
*/


#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#include <globals.h>
#define SIGN(A) (((A) > 0) ? (1) : ((((A) < 0) ? (-1) : (0))))

//-------------------------------------------------------------     DECLARATIONS

int32 filter_i1(int32 value);
int32 filter_i2(int32 value);

int32 filter_vel_1(int32 value);
int32 filter_vel_2(int32 value);
int32 filter_vel_3(int32 value);

uint8 LCRChecksum(uint8 *data_array, uint8 data_length);

uint8 check_enc_data(uint32*);

void reset_counters(void);


#endif

/* [] END OF FILE */