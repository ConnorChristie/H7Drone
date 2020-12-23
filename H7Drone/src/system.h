#pragma once

#include "platform.h"
#include <stdint.h>
#include <cmsis_compiler.h>

typedef void(*interruptFuncPtr)(void *ctx);

typedef struct
{
	interruptFuncPtr callback;
	void *data;
} interruptContext_t;

void enableInterrupt(u8 group, interruptFuncPtr fn, void *ctx);

#if defined(STM32F3) || defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
// See "RM CoreSight Architecture Specification"
// B2.3.10  "LSR and LAR, Software Lock Status Register and Software Lock Access Register"
// "E1.2.11  LAR, Lock Access Register"

#define DWT_LAR_UNLOCK_VALUE 0xC5ACCE55

#endif

// BASEPRI manipulation functions
// only set_BASEPRI is implemented in device library. It does always create memory barrier
// missing versions are implemented here

// set BASEPRI register, do not create memory barrier
__attribute__((always_inline)) static inline void __set_BASEPRI_nb(u32 basePri)
{
	__ASM volatile("\tMSR basepri, %0\n" : : "r" (basePri));
}

// set BASEPRI_MAX register, do not create memory barrier
__attribute__((always_inline)) static inline void __set_BASEPRI_MAX_nb(u32 basePri)
{
	__ASM volatile("\tMSR basepri_max, %0\n" : : "r" (basePri));
}

// restore BASEPRI (called as cleanup function), with global memory barrier
static inline void __basepriRestoreMem(u8 *val)
{
	__set_BASEPRI(*val);
}

// set BASEPRI_MAX, with global memory barrier, returns true
static inline u8 __basepriSetMemRetVal(u8 prio)
{
	__set_BASEPRI_MAX(prio);
	return 1;
}

// restore BASEPRI (called as cleanup function), no memory barrier
static inline void __basepriRestore(u8 *val)
{
	__set_BASEPRI_nb(*val);
}

// set BASEPRI_MAX, no memory barrier, returns true
static inline u8 __basepriSetRetVal(u8 prio)
{
	__set_BASEPRI_MAX_nb(prio);
	return 1;
}

// Run block with elevated BASEPRI (using BASEPRI_MAX), restoring BASEPRI on exit.
// All exit paths are handled. Implemented as for loop, does intercept break and continue
// Full memory barrier is placed at start and at exit of block
// __unused__ attribute is used to supress CLang warning
#define ATOMIC_BLOCK(prio) for ( u8 __basepri_save __attribute__ ((__cleanup__ (__basepriRestoreMem), __unused__)) = __get_BASEPRI(), \
                                     __ToDo = __basepriSetMemRetVal(prio); __ToDo ; __ToDo = 0 )

#define DISCARD(x) (void)x

void SysTick_Handler(void);
void cycleCounterInit(void);
u32 micros(void);
u32 microsISR(void);
void delayMicroseconds(u32 us);
void delay(u32 ms);

// IO
int GPIOPinIdx(u16 pin);
