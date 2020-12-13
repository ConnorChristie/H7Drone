#pragma once

// STM32H7
#define MAX_MPU_REGIONS 16

void memProtConfigure(void);
void memProtReset(void);
void initialiseMemorySections(void);
