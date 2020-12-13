#pragma once

#include <stm32h7xx_hal.h>
#include <stdint.h>
#include <stdbool.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

#if 0

#define NOINLINE             __attribute__ ((noinline))

#define FAST_CODE            __attribute__ ((section(".tcm_code")))
#define FAST_CODE_NOINLINE   NOINLINE

#define FAST_DATA_ZERO_INIT  __attribute__ ((section(".fastram_bss"), aligned(4)))
#define FAST_DATA            __attribute__ ((section(".fastram_data"), aligned(4)))

#else

#define NOINLINE

#define FAST_CODE
#define FAST_CODE_NOINLINE

#define FAST_DATA_ZERO_INIT
#define FAST_DATA

#endif

#define DMA_RAM              __attribute__ ((section(".DMA_RAM")))
#define DMA_RW_AXI           __attribute__ ((section(".DMA_RW_AXI")))
