#include "spi.h"
#include "system.h"

#include "stm32h7xx_ll_gpio.h"

uint8_t spiTransferByte(spiInstance_t *spi, uint8_t txByte)
{
	uint8_t in;

	spiTransfer(spi, &txByte, &in, 1);
	return in;
}

bool spiTransfer(spiInstance_t *spi, const uint8_t *txData, uint8_t *rxData, int len)
{
	HAL_StatusTypeDef status;

	if (!rxData)
	{
		// Tx only
		status = HAL_SPI_Transmit(&spi->instance, txData, len, 10);
	}
	else if (!txData)
	{
		// Rx only
		status = HAL_SPI_Receive(&spi->instance, rxData, len, 10);
	}
	else
	{
		// Tx and Rx
		status = HAL_SPI_TransmitReceive(&spi->instance, txData, rxData, len, 10);
	}

	if (status != HAL_OK)
	{
		return false;
	}

	return true;
}

void spiBusWriteRegister(spiInstance_t* spiInstance, uint8_t reg, uint8_t data)
{
	csLow(spiInstance);
	spiTransferByte(spiInstance, reg);
	spiTransferByte(spiInstance, data);
	csHigh(spiInstance);
}

bool spiBusWriteRegisterVerify(spiInstance_t* spiInstance, uint8_t reg, uint8_t data)
{
	spiBusWriteRegister(spiInstance, reg, data);
	delayMicroseconds(100);

	uint8_t attemptsRemaining = 20;
	while (attemptsRemaining--)
	{
		uint8_t in;
		spiBusReadRegisterBuffer(spiInstance, reg, &in, 1);
		if (in == data)
		{
			return true;
		}
		else
		{
			spiBusWriteRegister(spiInstance, reg, data);
			delayMicroseconds(100);
		}
	}
	return false;
}

uint8_t spiBusReadRegister(spiInstance_t *spi, uint8_t reg)
{
	uint8_t data;

	csLow(spi);
	spiTransferByte(spi, reg | 0x80);
	spiTransfer(spi, NULL, &data, 1);
	csHigh(spi);

	return data;
}

void spiBusReadRegisterBuffer(spiInstance_t *spi, uint8_t reg, uint8_t *data, uint8_t length)
{
	csLow(spi);
	spiTransferByte(spi, reg | 0x80);
	spiTransfer(spi, NULL, data, length);
	csHigh(spi);
}

void spiBusReadBuffer(spiInstance_t *spi, uint8_t *txData, uint8_t *rxData, uint8_t length)
{
	csLow(spi);
	spiTransfer(spi, txData, rxData, length);
	csHigh(spi);
}

void spiSetSpeed(spiInstance_t *spi, uint32_t speed)
{
	HAL_SPI_DeInit(&spi->instance);
	spi->instance.Init.BaudRatePrescaler = speed;
	HAL_SPI_Init(&spi->instance);
}

void csHigh(spiInstance_t* spi)
{
	LL_GPIO_SetOutputPin(spi->csPinPack, spi->csPin);
}

void csLow(spiInstance_t* spi)
{
	LL_GPIO_ResetOutputPin(spi->csPinPack, spi->csPin);
}
