#include "memprot.h"
#include "platform.h"

#include <string.h>

void memProtConfigure(void)
{
	HAL_MPU_Disable();

	MPU_Region_InitTypeDef MPU_InitStruct;
	MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
	MPU_InitStruct.SubRegionDisable = 0x00;
	MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;

	MPU_InitStruct.Number           = 0;
	MPU_InitStruct.BaseAddress      = 0x00000000;
	MPU_InitStruct.Size             = MPU_REGION_SIZE_64KB;
	MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RO_URO;
	MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
	MPU_InitStruct.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
	MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsBufferable     = MPU_ACCESS_BUFFERABLE;
	HAL_MPU_ConfigRegion(&MPU_InitStruct);

	MPU_InitStruct.Number           = 1;
	MPU_InitStruct.BaseAddress      = 0x30000000;
	MPU_InitStruct.Size             = 0x0d;
	MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
	MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_ENABLE;
	MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.IsCacheable      = MPU_ACCESS_CACHEABLE;
	MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
	HAL_MPU_ConfigRegion(&MPU_InitStruct);

	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void memProtReset(void)
{
	MPU_Region_InitTypeDef MPU_InitStruct;

	/* Disable the MPU */
	HAL_MPU_Disable();

	// Disable existing regions

	for (u8 region = 0; region <= MAX_MPU_REGIONS; region++)
	{
		MPU_InitStruct.Enable = MPU_REGION_DISABLE;
		MPU_InitStruct.Number = region;
		HAL_MPU_ConfigRegion(&MPU_InitStruct);
	}

	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void initialiseMemorySections(void)
{
	/* Load functions into ITCM RAM */
	extern uint8_t tcm_code_start;
	extern uint8_t tcm_code_end;
	extern uint8_t tcm_code;
	memcpy(&tcm_code_start, &tcm_code, (size_t)(&tcm_code_end - &tcm_code_start));

	/* Load FAST_DATA variable intializers into DTCM RAM */
	extern uint8_t _sfastram_data;
	extern uint8_t _efastram_data;
	extern uint8_t _sfastram_idata;
	memcpy(&_sfastram_data, &_sfastram_idata, (size_t)(&_efastram_data - &_sfastram_data));
}
