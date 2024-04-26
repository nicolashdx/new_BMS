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

    uint16_t maxCellVoltage;
    uint16_t minCellVoltage;
    uint16_t deltaVoltage;
} Master;

void BMS_Init(Master **BMS);
void Monitoring(Master *BMS);
void ElectricalManagement(Master *BMS);

#endif
