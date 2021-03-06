#pragma once

#include "platform.h"

typedef struct
{
	GPIO_TypeDef *csPinPack;
	u32 csPin;

	GPIO_TypeDef *extiPinPack;
	u32 extiPin;

	SPI_HandleTypeDef instance;
} spiInstance_t;

uint8_t spiTransferByte(spiInstance_t *spi, uint8_t txByte);
bool spiTransfer(spiInstance_t *spi, const uint8_t *txData, uint8_t *rxData, int len);

void spiBusWriteRegister(spiInstance_t* spiInstance, uint8_t reg, uint8_t data);
bool spiBusWriteRegisterVerify(spiInstance_t* spiInstance, uint8_t reg, uint8_t data);

uint8_t spiBusReadRegister(spiInstance_t *spi, uint8_t reg);
void spiBusReadRegisterBuffer(spiInstance_t *spi, uint8_t reg, uint8_t *data, uint8_t length);
void spiBusReadBuffer(spiInstance_t *spi, const uint8_t *txData, uint8_t *rxData, uint8_t length);

void spiSetSpeed(spiInstance_t *spi, uint32_t speed);

void csHigh(spiInstance_t* spi);
void csLow(spiInstance_t* spi);
