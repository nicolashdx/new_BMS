/*
 * bms.c
 *
 *  Created on: Jan 22, 2024
 *      Author: Nicolas
 */

#include "main.h"
#include "defines.h"
#include "bms.h"
#include "ltc.h"


void BMS_Init(Master **BMS) {
	*BMS = (Master*) calloc(1, sizeof(Master));
	LTC_config* config = (LTC_config*) calloc(1, sizeof(LTC_config));
	config->command = (LTC_command*) calloc(1 ,sizeof(LTC_command));
	LTC_Init(config);

	for(uint8_t i = 0; i < NUM_SLAVES; i++) {
		(*BMS)->slaves[i].config = config;
		(*BMS)->slaves[i].sensor.ADDR = i;
	}

	LTC_PEC_InitTable();
}

void BMS_Monitoring(Master *BMS) {
	LTC_SendBroadcastCommand(BMS->slaves[0].config, LTC_COMMAND_ADCV);
	uint16_t aux_minCellVoltage = UINT16_MAX;
	uint16_t aux_maxCellVoltage = 0;
	for(uint8_t i = 0; i < NUM_SLAVES; i++) {
		LTC_Read(LTC_READ_CELL, BMS->config, BMS->slaves[i]);
		if(BMS->slaves[i]->V_MIN < aux_minCellVoltage)
			aux_minCellVoltage = BMS->slaves[i]->V_MIN;
		if(BMS->slaves[i]->V_MAX > aux_maxCellVoltage)
			aux_maxCellVoltage = BMS->slaves[i]->V_MAX;
	}
	BMS->maxCellVoltage = aux_maxCellVoltage;
	BMS->minCellVoltage = aux_minCellVoltage;
	BMS->deltaVoltage = BMS->maxCellVoltage - BMS->minCellVoltage;
}
