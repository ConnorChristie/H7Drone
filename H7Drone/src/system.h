#pragma once

#include <stdint.h>
#include <cmsis_compiler.h>

typedef void(*interruptFuncPtr)(void *ctx);

typedef struct
{
	interruptFuncPtr callback;
	void *data;
} interruptContext_t;

void enableInterrupt(uint8_t group, interruptFuncPtr fn, void *ctx);

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
__attribute__((always_inline)) static inline void __set_BASEPRI_nb(uint32_t basePri)
{
	__ASM volatile("\tMSR basepri, %0\n" : : "r" (basePri));
}

// set BASEPRI_MAX register, do not create memory barrier
__attribute__((always_inline)) static inline void __set_BASEPRI_MAX_nb(uint32_t basePri)
{
	__ASM volatile("\tMSR basepri_max, %0\n" : : "r" (basePri));
}

// restore BASEPRI (called as cleanup function), with global memory barrier
static inline void __basepriRestoreMem(uint8_t *val)
{
	__set_BASEPRI(*val);
}

// set BASEPRI_MAX, with global memory barrier, returns true
static inline uint8_t __basepriSetMemRetVal(uint8_t prio)
{
	__set_BASEPRI_MAX(prio);
	return 1;
}

// restore BASEPRI (called as cleanup function), no memory barrier
static inline void __basepriRestore(uint8_t *val)
{
	__set_BASEPRI_nb(*val);
}

// set BASEPRI_MAX, no memory barrier, returns true
static inline uint8_t __basepriSetRetVal(uint8_t prio)
{
	__set_BASEPRI_MAX_nb(prio);
	return 1;
}

// Run block with elevated BASEPRI (using BASEPRI_MAX), restoring BASEPRI on exit.
// All exit paths are handled. Implemented as for loop, does intercept break and continue
// Full memory barrier is placed at start and at exit of block
// __unused__ attribute is used to supress CLang warning
#define ATOMIC_BLOCK(prio) for ( uint8_t __basepri_save __attribute__ ((__cleanup__ (__basepriRestoreMem), __unused__)) = __get_BASEPRI(), \
                                     __ToDo = __basepriSetMemRetVal(prio); __ToDo ; __ToDo = 0 )

#define NVIC_PRIORITY_GROUPING NVIC_PRIORITYGROUP_2
#define NVIC_BUILD_PRIORITY(base,sub) (((((base)<<(4-(7-(NVIC_PRIORITY_GROUPING))))|((sub)&(0x0f>>(7-(NVIC_PRIORITY_GROUPING)))))<<4)&0xf0)
#define NVIC_PRIO_MAX NVIC_BUILD_PRIORITY(0, 1)

#define DISCARD(x) (void)x

void SysTick_Handler(void);
void cycleCounterInit(void);
uint32_t micros(void);
void delayMicroseconds(uint32_t us);
void delay(uint32_t ms);

// IO
int GPIOPinIdx(uint16_t pin);
