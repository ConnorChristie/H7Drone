#include "sbus.h"
#include "system.h"
#include "rcc.h"
#include "nvic.h"

#include "stm32h7xx_ll_usart.h"

sbusFrameData_t sbusFrameData;

static void sbusDataReceive(u16 c, void *data)
{
	sbusFrameData_t *sbusFrameData = (sbusFrameData_t *)data;

	const timeUs_t nowUs = microsISR();
	const timeDelta_t sbusFrameTime = cmpTimeUs(nowUs, sbusFrameData->startAtUs);

	if (sbusFrameTime > (long)(SBUS_TIME_NEEDED_PER_FRAME + 500))
	{
		sbusFrameData->position = 0;
	}

	if (sbusFrameData->position == 0)
	{
		if (c != SBUS_FRAME_BEGIN_BYTE)
		{
			return;
		}
		sbusFrameData->startAtUs = nowUs;
	}

	if (sbusFrameData->position < SBUS_FRAME_SIZE)
	{
		sbusFrameData->frame.bytes[sbusFrameData->position++] = (u8)c;
		if (sbusFrameData->position < SBUS_FRAME_SIZE)
		{
			sbusFrameData->done = false;
		}
		else
		{
			sbusFrameData->done = true;
		}
	}
}

void sbusInit(rxConfig_t rxConfig)
{
	RCC_ClockCmd(RCC_APB2(USART6), ENABLE);

	GPIO_InitTypeDef init;
	init.Pin = GPIO_PIN_7;
	init.Mode = GPIO_MODE_AF_PP;
	init.Speed = GPIO_SPEED_FREQ_LOW;
	init.Pull = GPIO_NOPULL;
	init.Alternate = GPIO_AF7_USART6;
	HAL_GPIO_Init(GPIOC, &init);

	NVIC_SetPriority(USART6_IRQn, NVIC_PRIO_SERIALUART6);
	NVIC_EnableIRQ(USART6_IRQn);

	UART_HandleTypeDef uart = {};
	uart.Instance = USART6;
	HAL_UART_DeInit(&uart);

	uart.Init.BaudRate = 100000;
	uart.Init.Mode = UART_MODE_RX;
	uart.Init.Parity = UART_PARITY_EVEN;
	uart.Init.StopBits = USART_STOPBITS_2;
	uart.Init.WordLength = UART_WORDLENGTH_9B;
	uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;

	uart.AdvancedInit.AdvFeatureInit |= UART_ADVFEATURE_RXINVERT_INIT;
	uart.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;

	HAL_UART_Init(&uart);

	/* Enable the UART Parity Error Interrupt */
	LL_USART_EnableIT_PE(uart.Instance);

	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	LL_USART_EnableIT_ERROR(uart.Instance);

	/* Enable the UART Data Register not empty Interrupt */
	LL_USART_EnableIT_RXNE_RXFNE(uart.Instance);

	/* Enable Idle Line detection */
	LL_USART_EnableIT_IDLE(uart.Instance);
}

void uartIrqHandler(USART_TypeDef *uart)
{
	if (LL_USART_IsActiveFlag_RXNE_RXFNE(uart))
	{
		u8 rbyte = LL_USART_ReceiveData8(uart);
		sbusDataReceive(rbyte, &sbusFrameData);

		/* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
		LL_USART_DisableIT_PE(uart);
		LL_USART_DisableIT_ERROR(uart);
		LL_USART_RequestRxDataFlush(uart);
	}

	/* UART parity error interrupt occurred -------------------------------------*/
	if (LL_USART_IsActiveFlag_PE(uart))
		LL_USART_ClearFlag_PE(uart);

	/* UART frame error interrupt occurred --------------------------------------*/
	if (LL_USART_IsActiveFlag_FE(uart))
		LL_USART_ClearFlag_FE(uart);

	/* UART noise error interrupt occurred --------------------------------------*/
	if (LL_USART_IsActiveFlag_NE(uart))
		LL_USART_ClearFlag_NE(uart);

	/* UART Over-Run interrupt occurred -----------------------------------------*/
	if (LL_USART_IsActiveFlag_ORE(uart))
		LL_USART_ClearFlag_ORE(uart);

	if (LL_USART_IsActiveFlag_IDLE(uart))
	{
		// Call idle callback
		
		LL_USART_ClearFlag_IDLE(uart);
	}
}

#define UART_IRQHandler(type, number)    \
    void type ## number ## _IRQHandler(void)  \
    {                                         \
        uartIrqHandler(type ## number);       \
    }

UART_IRQHandler(USART, 6)
