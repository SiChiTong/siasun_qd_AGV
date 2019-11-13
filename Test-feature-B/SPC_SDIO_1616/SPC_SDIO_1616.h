/*
 * SPC_SDIO_1616.h
 *
 *  Created on: 2019-8-26
 *      Author: siasunggm
 */

#ifndef SPC_SDIO_1616_H_
#define SPC_SDIO_1616_H_

#include <can.h>
#include <stdint.h>

typedef union
{
	 uint8_t SDIO_Out[4];
	 __u8  SDIO_Idata[4];
}AGV_SDIO_Data;

typedef struct SPC_SDIO_1616_a
{
	uint8_t Q_Out[32];
}AGV_SDIO_Data_Out;

typedef struct SPC_SDIO_1616
{
	 uint16_t point[32];
}SPC_SDIO_1616_Data;

typedef struct SPC_SDIO_Sound_Data
{
	 uint8_t Sound_Control[4];
}Sound_Data;

typedef struct SPC_SDIO_Lamplight_Data
{
	 uint8_t Blue_Light;
	 uint8_t Green_Light;
	 uint8_t Red_Light;
}Lamplight_Data;

void SPC_SDIO_1616a_Analyze_Data();
void SPC_SDIO_1616c_Analyze_Data();
void SDIO_Out_Data();

#endif /* SPC_SDIO_1616_H_ */
