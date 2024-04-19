/*
 * bms.h
 *
 *  Created on: Jan 21, 2024
 *      Author: Nicolas
 */

#ifndef BMS_H
#define BMS_H

#include "stdlib.h"
#include "ltc.h"
#include "defines.h"

typedef struct Master {
    Slave slaves[NUM_SLAVES];
} Master;

void BMS_Init(Master **BMS);
void BMS_Monitoring(Master *BMS);

#endif
