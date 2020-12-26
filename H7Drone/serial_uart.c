#include "serial_uart.h"
#include "nvic.h"

#include "stm32h7xx_ll_usart.h"

#ifdef STM32H7
uartDevice_s uartDevmap[UARTDEV_COUNT] = {
	[UARTDEV_1] = { .ref = USART1, .IRQn = USART1_IRQn, .rcc = RCC_APB2(USART1), .rxPriority = NVIC_PRIO_SERIALUART1 },
	[UARTDEV_2] = { .ref = USART2, .IRQn = USART2_IRQn, .rcc = RCC_APB1L(USART2), .rxPriority = NVIC_PRIO_SERIALUART2 },
	[UARTDEV_3] = { .ref = USART3, .IRQn = USART3_IRQn, .rcc = RCC_APB1L(USART3), .rxPriority = NVIC_PRIO_SERIALUART3 },
	[UARTDEV_4] = { .ref = UART4, .IRQn = UART4_IRQn, .rcc = RCC_APB1L(UART4), .rxPriority = NVIC_PRIO_SERIALUART4 },
	[UARTDEV_5] = { .ref = UART5, .IRQn = UART5_IRQn, .rcc = RCC_APB1L(UART5), .rxPriority = NVIC_PRIO_SERIALUART5 },
	[UARTDEV_6] = { .ref = USART6, .IRQn = USART6_IRQn, .rcc = RCC_APB2(USART6), .rxPriority = NVIC_PRIO_SERIALUART6 },
	[UARTDEV_7] = { .ref = UART7, .IRQn = UART7_IRQn, .rcc = RCC_APB1L(UART7), .rxPriority = NVIC_PRIO_SERIALUART7 },
	[UARTDEV_8] = { .ref = UART8, .IRQn = UART8_IRQn, .rcc = RCC_APB1L(UART8), .rxPriority = NVIC_PRIO_SERIALUART8 }
};
#endif

void uartInit(uartIdentifier_e dev, serialPort_s port)
{
	uartDevice_s *device = &uartDevmap[dev];
	device->port = port;

	RCC_ClockCmd(device->rcc, ENABLE);

	GPIO_InitTypeDef init;
	init.Pin = GPIO_PIN_7;
	init.Mode = GPIO_MODE_AF_PP;
	init.Speed = GPIO_SPEED_FREQ_LOW;
	init.Pull = GPIO_NOPULL;
	init.Alternate = GPIO_AF7_USART6;
	HAL_GPIO_Init(GPIOC, &init);

	HAL_NVIC_SetPriority(device->IRQn, NVIC_PRIORITY_BASE(device->rxPriority), NVIC_PRIORITY_SUB(device->rxPriority));
	HAL_NVIC_EnableIRQ(device->IRQn);

	UART_HandleTypeDef uart = { };
	uart.Instance = device->ref;
	HAL_UART_DeInit(&uart);

	uart.Init = port.init;
	uart.AdvancedInit = port.advInit;

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

void uartIrqHandler(uartDevice_s *uart)
{
	if (LL_USART_IsActiveFlag_RXNE_RXFNE(uart->ref))
	{
		u8 rbyte = LL_USART_ReceiveData8(uart->ref);
		uart->port.rxCallback(rbyte, uart->port.rxCallbackData);

		/* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
		LL_USART_DisableIT_PE(uart->ref);
		LL_USART_DisableIT_ERROR(uart->ref);
		LL_USART_RequestRxDataFlush(uart->ref);
	}

	/* UART parity error interrupt occurred -------------------------------------*/
	if (LL_USART_IsActiveFlag_PE(uart->ref))
		LL_USART_ClearFlag_PE(uart->ref);

	/* UART frame error interrupt occurred --------------------------------------*/
	if (LL_USART_IsActiveFlag_FE(uart->ref))
		LL_USART_ClearFlag_FE(uart->ref);

	/* UART noise error interrupt occurred --------------------------------------*/
	if (LL_USART_IsActiveFlag_NE(uart->ref))
		LL_USART_ClearFlag_NE(uart->ref);

	/* UART Over-Run interrupt occurred -----------------------------------------*/
	if (LL_USART_IsActiveFlag_ORE(uart->ref))
		LL_USART_ClearFlag_ORE(uart->ref);

	if (LL_USART_IsActiveFlag_IDLE(uart->ref))
	{
		// Call idle callback
		
		LL_USART_ClearFlag_IDLE(uart->ref);
	}
}

#define UART_IRQHandler(type, number, dev)             \
    void type ## number ## _IRQHandler(void)           \
    {                                                  \
        uartIrqHandler(&uartDevmap[UARTDEV_ ## dev]);  \
    }

UART_IRQHandler(USART, 1, 1)
UART_IRQHandler(USART, 2, 2)
UART_IRQHandler(USART, 3, 3)
UART_IRQHandler(UART, 4, 4)
UART_IRQHandler(UART, 5, 5)
UART_IRQHandler(USART, 6, 6)
UART_IRQHandler(UART, 7, 7)
UART_IRQHandler(UART, 8, 8)
