#include "main.h"

SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
LR1121_HandleTypeDef hlr1121;

int main(void)
{
	HAL_StatusTypeDef st;
	const uint8_t demo_payload[] = {'H', 'I', '-', 'L', 'R', '1', '1', '2', '1'};
	const LR1121_LoRaProfile demo_profile = {
		//.frequency_hz = 2403000000UL,
		.frequency_hz = 868030000UL,
		.modulation = {
			.sf = LR1121_LORA_SF12,
			.bw = LR1121_LORA_BW_125,
			.cr = LR1121_LORA_CR_4_5,
			.ldro = LR1121_LORA_LDRO_OFF,
		},
		.packet = {
			.preamble_len = 8,
			.header_type = LR1121_LORA_HEADER_EXPLICIT,
			.payload_len = (uint8_t)sizeof(demo_payload),
			.crc = LR1121_LORA_CRC_ON,
			.iq = LR1121_LORA_IQ_STANDARD,
		},
		.tx = {
			.power_dbm = 22,
			.ramp = LR1121_RAMP_48_US,
		},
	};

	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_USART2_UART_Init();

	hlr1121.hspi = &hspi1;
	hlr1121.reset_port = LR1121_RESET_PORT;
	hlr1121.reset_pin = LR1121_RESET_PIN;
	hlr1121.cs_port = LR1121_CS_PORT;
	hlr1121.cs_pin = LR1121_CS_PIN;
	hlr1121.busy_port = LR1121_BUSY_PORT;
	hlr1121.busy_pin = LR1121_BUSY_PIN;
	hlr1121.dio1_port = LR1121_DIO1_PORT;
	hlr1121.dio1_pin = LR1121_DIO1_PIN;

	st = LR1121_Init(&hlr1121);
	if (st != HAL_OK)
	{
		uart_log("LR1121 init failed: %d busy=%u dio1=%u\r\n",
				 (int)st,
				 (unsigned int)HAL_GPIO_ReadPin(LR1121_BUSY_PORT, LR1121_BUSY_PIN),
				 (unsigned int)HAL_GPIO_ReadPin(LR1121_DIO1_PORT, LR1121_DIO1_PIN));
		Error_Handler();
	}

	st = LR1121_ConfigureLoRa(&hlr1121, &demo_profile);
	if (st != HAL_OK)
	{
		const LR1121_DebugInfo *dbg = LR1121_GetLastDebugInfo();
		uint16_t sys_err = 0U;
		(void)LR1121_GetErrors(&hlr1121, &sys_err);
		uart_log("LR1121 LoRa cfg failed: %d stage=%u op=0x%04X irq=0x%08lX sys_err=0x%04X\r\n",
				 (int)st,
				 (unsigned int)dbg->stage,
				 (unsigned int)dbg->opcode,
				 dbg->irq,
				 (unsigned int)sys_err);
		Error_Handler();
	}

	uart_log("LR1121 ready, sending demo LoRa packet\r\n");

	while (1)
	{
		const uint32_t tx_period_ms = 100U;
		uint32_t loop_start_ms = HAL_GetTick();
		uint32_t irq_mask = 0U;

		HAL_GPIO_WritePin(LR1121_TX_IND_PORT, LR1121_TX_IND_PIN, GPIO_PIN_SET);

		st = LR1121_SendLoRaPacket(&hlr1121,
											 demo_payload,
											 (uint8_t)sizeof(demo_payload),
											 0x00FFFF);
		if (st == HAL_OK)
		{
			uart_log("Demo LoRa packet queued\r\n");
		}
		else
		{
			const LR1121_DebugInfo *dbg = LR1121_GetLastDebugInfo();
			uint16_t sys_err = 0U;
			(void)LR1121_GetIrqStatus(&hlr1121, &irq_mask);
			(void)LR1121_GetErrors(&hlr1121, &sys_err);
			uart_log("Demo LoRa packet failed: %d irq=0x%08lX busy=%u dio1=%u stage=%u op=0x%04X dbg_irq=0x%08lX sys_err=0x%04X\r\n",
					 (int)st,
					 irq_mask,
					 (unsigned int)HAL_GPIO_ReadPin(LR1121_BUSY_PORT, LR1121_BUSY_PIN),
					 (unsigned int)HAL_GPIO_ReadPin(LR1121_DIO1_PORT, LR1121_DIO1_PIN),
					 (unsigned int)dbg->stage,
					 (unsigned int)dbg->opcode,
					 dbg->irq,
					 (unsigned int)sys_err);
		}

		HAL_GPIO_WritePin(LR1121_TX_IND_PORT, LR1121_TX_IND_PIN, GPIO_PIN_RESET);

		HAL_Delay(tx_period_ms);
	}
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
																RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

void Error_Handler(void)
{
	__disable_irq();
	while (1)
	{
	}
}

void SysTick_Handler(void)
{
	HAL_IncTick();
}

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
	while (1)
	{
	}
}

void MemManage_Handler(void)
{
	while (1)
	{
	}
}

void BusFault_Handler(void)
{
	while (1)
	{
	}
}

void UsageFault_Handler(void)
{
	while (1)
	{
	}
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}
