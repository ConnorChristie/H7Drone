#include <stm32h7xx_hal.h>
#include <stm32_hal_legacy.h>
#include <SysprogsProfiler.h>
#include <stdlib.h>
#include <string.h>

#include "system.h"
#include "scheduler.h"
#include "imu.h"
#include "dshot.h"

static void SPI1_Init(void);
static void SPI4_Init(void);
void SystemClock_Config(void);
void initMotors(void);

#define VECT_TAB_OFFSET  0x00

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	InitializeInstrumentingProfiler();
	InitializeSamplingProfiler();

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

	// Enable CPU L1-Cache
	SCB_EnableICache();
	SCB_EnableDCache();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	cycleCounterInit();

	SPI1_Init();
	//SPI4_Init();
	initMotors();

	schedulerInit();
	schedulerSetCalulateTaskStatistics(true);
	setTaskEnabled(TASK_GYRO, true);
	setTaskEnabled(TASK_FILTER, false);
	setTaskEnabled(TASK_ACCEL, true);
	setTaskEnabled(TASK_COMPASS, false);
	setTaskEnabled(TASK_FLIGHT, true);
	setTaskEnabled(TASK_DSHOT, true);
	setTaskEnabled(TASK_LED, true);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Pin = GPIO_PIN_3;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);

	while (1)
	{
		scheduler();
	}
}

void initMotors(void)
{
	motorInstance_t motor = {
		.timer = { 
			.pinPack = GPIOB,
			.pin = GPIO_PIN_1,
			.alternateFunction = GPIO_AF2_TIM3,
			.instance = TIM3,
			.channel = TIM_CHANNEL_4
		},
		.dma = {
			.instance = DMA1,
			.stream = LL_DMA_STREAM_2,
			.channel = LL_DMAMUX1_REQ_TIM3_CH4
		}
	};

	dshotInit(0, motor);
}

static void SPI1_Init(void)
{
	spiInstance_t spi = { 
		.csPinPack = GPIOC,
		.csPin = GPIO_PIN_15,
		
		.extiPinPack = GPIOB,
		.extiPin = GPIO_PIN_2
	};

	__HAL_RCC_SPI1_CLK_ENABLE();
	__HAL_RCC_SPI1_RELEASE_RESET();

	GPIO_InitTypeDef init;

	// SCK
	init.Pin = GPIO_PIN_5;
	init.Mode = GPIO_MODE_AF_PP;
	init.Speed = GPIO_SPEED_FAST;
	init.Pull = GPIO_PULLDOWN;
	init.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &init);

	// MISO
	init.Pin = GPIO_PIN_6;
	init.Mode = GPIO_MODE_AF_PP;
	init.Speed = GPIO_SPEED_FAST;
	init.Pull = GPIO_PULLDOWN;
	init.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &init);

	// MOSI
	init.Pin = GPIO_PIN_7;
	init.Mode = GPIO_MODE_AF_PP;
	init.Speed = GPIO_SPEED_FAST;
	init.Pull = GPIO_PULLDOWN;
	init.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOD, &init);

	// CS
	init.Pin = spi.csPin;
	init.Mode = GPIO_MODE_OUTPUT_PP;
	init.Speed = GPIO_SPEED_FREQ_HIGH;
	init.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(spi.csPinPack, &init);

	// EXTI
	init.Pin = spi.extiPin;
	init.Mode = GPIO_MODE_INPUT | GPIO_MODE_IT_RISING;
	init.Speed = GPIO_SPEED_FREQ_LOW;
	init.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(spi.extiPinPack, &init);

	spi.instance.Instance = SPI1;
	spi.instance.Init.Mode = SPI_MODE_MASTER;
	spi.instance.Init.Direction = SPI_DIRECTION_2LINES;
	spi.instance.Init.DataSize = SPI_DATASIZE_8BIT;
	spi.instance.Init.CLKPolarity = SPI_POLARITY_LOW;
	spi.instance.Init.CLKPhase = SPI_PHASE_1EDGE;
	spi.instance.Init.NSS = SPI_NSS_SOFT;
	spi.instance.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	spi.instance.Init.FirstBit = SPI_FIRSTBIT_MSB;
	spi.instance.Init.TIMode = SPI_TIMODE_DISABLE;
	spi.instance.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	spi.instance.Init.CRCPolynomial = 7;

	HAL_SPI_Init(&spi.instance);

	imuInit(spi, 0);
}

static void SPI4_Init(void)
{
	spiInstance_t spi = { 
		.csPinPack = GPIOE,
		.csPin = GPIO_PIN_11,

		.extiPinPack = GPIOE,
		.extiPin = GPIO_PIN_15
	};

	__HAL_RCC_SPI4_CLK_ENABLE();
	__HAL_RCC_SPI4_RELEASE_RESET();

	spi.instance.Instance = SPI4;
	HAL_SPI_DeInit(&spi.instance);

	GPIO_InitTypeDef init;

	// SCK
	init.Pin = GPIO_PIN_12;
	init.Mode = GPIO_MODE_AF_PP;
	init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	init.Pull = GPIO_PULLDOWN;
	init.Alternate = GPIO_AF5_SPI4;
	HAL_GPIO_Init(GPIOE, &init);

	// MISO
	init.Pin = GPIO_PIN_13;
	init.Mode = GPIO_MODE_AF_PP;
	init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	init.Pull = GPIO_PULLUP;
	init.Alternate = GPIO_AF5_SPI4;
	HAL_GPIO_Init(GPIOE, &init);

	// MOSI
	init.Pin = GPIO_PIN_14;
	init.Mode = GPIO_MODE_AF_PP;
	init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	init.Pull = GPIO_NOPULL;
	init.Alternate = GPIO_AF5_SPI4;
	HAL_GPIO_Init(GPIOE, &init);

	// CS
	init.Pin = spi.csPin;
	init.Mode = GPIO_MODE_OUTPUT_PP;
	init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	init.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(spi.csPinPack, &init);

	// EXTI
	init.Pin = spi.extiPin;
	init.Mode = GPIO_MODE_INPUT | GPIO_MODE_IT_RISING;
	init.Speed = GPIO_SPEED_FREQ_LOW;
	init.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(spi.extiPinPack, &init);

	spi.instance.Init.Mode = SPI_MODE_MASTER;
	spi.instance.Init.Direction = SPI_DIRECTION_2LINES;
	spi.instance.Init.DataSize = SPI_DATASIZE_8BIT;
	spi.instance.Init.CLKPolarity = SPI_POLARITY_HIGH;
	spi.instance.Init.CLKPhase = SPI_PHASE_2EDGE;
	spi.instance.Init.NSS = SPI_NSS_SOFT;
	spi.instance.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	spi.instance.Init.FirstBit = SPI_FIRSTBIT_MSB;
	spi.instance.Init.TIMode = SPI_TIMODE_DISABLE;
	spi.instance.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	spi.instance.Init.CRCPolynomial = 7;
	spi.instance.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	spi.instance.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;

	HAL_SPI_Init(&spi.instance);

	imuInit(spi, 1);
}

static void Error_Handler(void)
{
	while (1) ;
}

typedef struct pllConfig_s {
	uint16_t clockMhz;
	uint8_t m;
	uint16_t n;
	uint8_t p;
	uint8_t q;
	uint8_t r;
	uint32_t vos;
} pllConfig_t;

/*
   PLL1 configuration for different silicon revisions.

   Note for future overclocking support.

   - Rev.Y (and Rev.X), nominal max at 400MHz, runs stably overclocked to 480MHz.
   - Rev.V, nominal max at 480MHz, runs stably at 540MHz, but not to 600MHz (VCO probably out of operating range)

   - A possible frequency table would look something like this, and a revision
     check logic would place a cap for Rev.Y and V.

        400 420 440 460 (Rev.Y & V ends here) 480 500 520 540
 */

// 400MHz for Rev.Y (and Rev.X)
pllConfig_t pll1ConfigRevY = {
	.clockMhz = 400,
	.m = 4,
	.n = 400,
	.p = 2,
	.q = 8,
	.r = 5,
	.vos = PWR_REGULATOR_VOLTAGE_SCALE1
};

// 480MHz for Rev.V
pllConfig_t pll1ConfigRevV = {
	.clockMhz = 480,
	.m = 4,
	.n = 480,
	.p = 2,
	.q = 8,
	.r = 5,
	.vos = PWR_REGULATOR_VOLTAGE_SCALE0
};

// HSE clock configuration, originally taken from
// STM32Cube_FW_H7_V1.3.0/Projects/STM32H743ZI-Nucleo/Examples/RCC/RCC_ClockConfig/Src/main.c

static void SystemClockHSE_Config(void)
{
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };

#ifdef notdef
	// CSI has been disabled at SystemInit().
	// HAL_RCC_ClockConfig() will fail because CSIRDY is off.

	/* -1- Select CSI as system clock source to allow modification of the PLL configuration */

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_CSI;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}
#endif

	pllConfig_t *pll1Config = (HAL_GetREVID() == REV_ID_V) ? &pll1ConfigRevV : &pll1ConfigRevY;

	// Configure voltage scale.
	// It has been pre-configured at PWR_REGULATOR_VOLTAGE_SCALE1,
	// and it may stay or overridden by PWR_REGULATOR_VOLTAGE_SCALE0 depending on the clock config.

	__HAL_PWR_VOLTAGESCALING_CONFIG(pll1Config->vos);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
		// Empty
	}

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;  // Even Nucleo-H473 work without RCC_HSE_BYPASS

	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = pll1Config->m;
	RCC_OscInitStruct.PLL.PLLN = pll1Config->n;
	RCC_OscInitStruct.PLL.PLLP = pll1Config->p;
	RCC_OscInitStruct.PLL.PLLQ = pll1Config->q;
	RCC_OscInitStruct.PLL.PLLR = pll1Config->r;

	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;

	HAL_StatusTypeDef status = HAL_RCC_OscConfig(&RCC_OscInitStruct);

	if (status != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	// Configure PLL2 and PLL3
	// Use of PLL2 and PLL3 are not determined yet.
	// A review of total system wide clock requirements is necessary.


	// Configure SCGU (System Clock Generation Unit)
	// Select PLL as system clock source and configure bus clock dividers.
	//
	// Clock type and divider member names do not have direct visual correspondence.
	// Here is how these correspond:
	//   RCC_CLOCKTYPE_SYSCLK           sys_ck
	//   RCC_CLOCKTYPE_HCLK             AHBx (rcc_hclk1,rcc_hclk2,rcc_hclk3,rcc_hclk4)
	//   RCC_CLOCKTYPE_D1PCLK1          APB3 (rcc_pclk3)
	//   RCC_CLOCKTYPE_PCLK1            APB1 (rcc_pclk1)
	//   RCC_CLOCKTYPE_PCLK2            APB2 (rcc_pclk2)
	//   RCC_CLOCKTYPE_D3PCLK1          APB4 (rcc_pclk4)

	RCC_ClkInitStruct.ClockType = (\
	    RCC_CLOCKTYPE_SYSCLK | \
	    RCC_CLOCKTYPE_HCLK | \
	    RCC_CLOCKTYPE_D1PCLK1 | \
	    RCC_CLOCKTYPE_PCLK1 | \
	    RCC_CLOCKTYPE_PCLK2  | \
	    RCC_CLOCKTYPE_D3PCLK1);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // = PLL1P = 400
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;  // = PLL1P(400) / 1 = 400
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;    // = SYSCLK(400) / 2 = 200
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;   // = HCLK(200) / 2 = 100
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;   // = HCLK(200) / 2 = 100
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;   // = HCLK(200) / 2 = 100
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;   // = HCLK(200) / 2 = 100

	// For HCLK=200MHz with VOS1 range, ST recommended flash latency is 2WS.
	// RM0433 (Rev.5) Table 12. FLASH recommended number of wait states and programming delay
	//
	// For higher HCLK frequency, VOS0 is available on RevV silicons, with FLASH wait states 4WS
	// AN5312 (Rev.1) Section 1.2.1 Voltage scaling Table.1

	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	/* -4- Optional: Disable CSI Oscillator (if the HSI is no more needed by the application)*/
	RCC_OscInitStruct.OscillatorType  = RCC_OSCILLATORTYPE_CSI;
	RCC_OscInitStruct.CSIState        = RCC_CSI_OFF;
	RCC_OscInitStruct.PLL.PLLState    = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}
}

void SystemClock_Config(void)
{
	// Configure power supply

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	// Pre-configure voltage scale to PWR_REGULATOR_VOLTAGE_SCALE1.
	// SystemClockHSE_Config may configure PWR_REGULATOR_VOLTAGE_SCALE0.

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
		// Empty
	}

	SystemClockHSE_Config();

	/*activate CSI clock mondatory for I/O Compensation Cell*/

	__HAL_RCC_CSI_ENABLE();

	/* Enable SYSCFG clock mondatory for I/O Compensation Cell */

	__HAL_RCC_SYSCFG_CLK_ENABLE();

	/* Enables the I/O Compensation Cell */

	HAL_EnableCompensationCell();

	// Configure peripheral clocks

	RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

	// Configure HSI48 as peripheral clock for USB

	RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	RCC_PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
	HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

	// Configure CRS for dynamic calibration of HSI48
	// While ES0392 Rev 5 "STM32H742xI/G and STM32H743xI/G device limitations" states CRS not working for REV.Y,
	// it is always turned on as it seems that it has no negative effect on clock accuracy.

	RCC_CRSInitTypeDef crsInit = {
		.Prescaler = RCC_CRS_SYNC_DIV1,
		.Source = RCC_CRS_SYNC_SOURCE_USB2,
		.Polarity = RCC_CRS_SYNC_POLARITY_RISING,
		.ReloadValue = RCC_CRS_RELOADVALUE_DEFAULT,
		.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT,
		.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT,
	};

	__HAL_RCC_CRS_CLK_ENABLE();
	HAL_RCCEx_CRSConfig(&crsInit);

#ifdef USE_CRS_INTERRUPTS
	// Turn on USE_CRS_INTERRUPTS to see CRS in action
	HAL_NVIC_SetPriority(CRS_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(CRS_IRQn);
	__HAL_RCC_CRS_ENABLE_IT(RCC_CRS_IT_SYNCOK | RCC_CRS_IT_SYNCWARN | RCC_CRS_IT_ESYNC | RCC_CRS_IT_ERR);
#endif

	// Configure UART peripheral clock sources
	//
	// Possible sources:
	//   D2PCLK1 (pclk1 for APB1 = USART234578)
	//   D2PCLK2 (pclk2 for APB2 = USART16)
	//   PLL2 (pll2_q_ck)
	//   PLL3 (pll3_q_ck),
	//   HSI (hsi_ck),
	//   CSI (csi_ck),LSE(lse_ck);

	RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART16 | RCC_PERIPHCLK_USART234578;
	RCC_PeriphClkInit.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
	RCC_PeriphClkInit.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
	HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

	// Configure SPI peripheral clock sources
	//
	// Possible sources for SPI123:
	//   PLL (pll1_q_ck)
	//   PLL2 (pll2_p_ck)
	//   PLL3 (pll3_p_ck)
	//   PIN (I2S_CKIN)
	//   CLKP (per_ck)
	// Possible sources for SPI45:
	//   D2PCLK1 (rcc_pclk2 = APB1) 100MHz
	//   PLL2 (pll2_q_ck)
	//   PLL3 (pll3_q_ck)
	//   HSI (hsi_ker_ck)
	//   CSI (csi_ker_ck)
	//   HSE (hse_ck)
	// Possible sources for SPI6:
	//   D3PCLK1 (rcc_pclk4 = APB4) 100MHz
	//   PLL2 (pll2_q_ck)
	//   PLL3 (pll3_q_ck)
	//   HSI (hsi_ker_ck)
	//   CSI (csi_ker_ck)
	//   HSE (hse_ck)

	// For the first cut, we use 100MHz from various sources

	RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SPI123 | RCC_PERIPHCLK_SPI45 | RCC_PERIPHCLK_SPI6;
	RCC_PeriphClkInit.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
	RCC_PeriphClkInit.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
	RCC_PeriphClkInit.Spi6ClockSelection = RCC_SPI6CLKSOURCE_D3PCLK1;
	HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

	// Configure I2C peripheral clock sources
	//
	// Current source for I2C123:
	//   D2PCLK1 (rcc_pclk1 = APB1 peripheral clock)
	//
	// Current source for I2C4:
	//   D3PCLK1 (rcc_pclk4 = APB4 peripheral clock)
	//
	// Note that peripheral clock determination in bus_i2c_hal_init.c must be modified when the sources are modified.

	RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C123 | RCC_PERIPHCLK_I2C4;
	RCC_PeriphClkInit.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
	RCC_PeriphClkInit.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
	HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

#ifdef USE_SDCARD_SDIO
	RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDMMC;
	RCC_PeriphClkInit.PLL2.PLL2M = 5;
	RCC_PeriphClkInit.PLL2.PLL2N = 500;
	RCC_PeriphClkInit.PLL2.PLL2P = 2;   // 500Mhz
	RCC_PeriphClkInit.PLL2.PLL2Q = 3;   // 266Mhz - 133Mhz can be derived from this for for QSPI if flash chip supports the speed.
	RCC_PeriphClkInit.PLL2.PLL2R = 4;   // 200Mhz HAL LIBS REQUIRE 200MHZ SDMMC CLOCK, see HAL_SD_ConfigWideBusOperation, SDMMC_HSpeed_CLK_DIV, SDMMC_NSpeed_CLK_DIV
	RCC_PeriphClkInit.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
	RCC_PeriphClkInit.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
	RCC_PeriphClkInit.PLL2.PLL2FRACN = 0;
	RCC_PeriphClkInit.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
	HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);
#endif

	// Configure MCO clocks for clock test/verification

	// Possible sources for MCO1:
	//   RCC_MCO1SOURCE_HSI (hsi_ck)
	//   RCC_MCO1SOURCE_LSE (?)
	//   RCC_MCO1SOURCE_HSE (hse_ck)
	//   RCC_MCO1SOURCE_PLL1QCLK (pll1_q_ck)
	//   RCC_MCO1SOURCE_HSI48 (hsi48_ck)

	//  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);     // HSE(8M) / 1 = 1M
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_4);       // HSI48(48M) / 4 = 12M

	// Possible sources for MCO2:
	//   RCC_MCO2SOURCE_SYSCLK  (sys_ck)
	//   RCC_MCO2SOURCE_PLL2PCLK (pll2_p_ck)
	//   RCC_MCO2SOURCE_HSE (hse_ck)
	//   RCC_MCO2SOURCE_PLLCLK (pll1_p_ck)
	//   RCC_MCO2SOURCE_CSICLK (csi_ck)
	//   RCC_MCO2SOURCE_LSICLK (lsi_ck)

	HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLLCLK, RCC_MCODIV_15);   // PLL1P(400M) / 15 = 26.67M
}
