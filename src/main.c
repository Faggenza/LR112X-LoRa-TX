#include "main.h"
#include "runtime_control.h"

SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;
LR1121_HandleTypeDef hlr1121;
static RuntimeControlCtx g_runtime;

int main(void)
{
	HAL_StatusTypeDef st;
	const LR1121_LoRaProfile default_profile = {
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
			.payload_len = 16,
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

	RuntimeControl_Init(&g_runtime, &hlr1121, &huart2, &default_profile);
	st = RuntimeControl_ApplyInitial(&g_runtime);
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

	RuntimeControl_PrintWelcome(&g_runtime);

	while (1)
	{
		uint32_t wait_start_ms;
		uint32_t irq_mask = 0U;
		uint8_t payload_len;
		uint32_t tx_period_ms;

		RuntimeControl_Poll(&g_runtime);
		payload_len = RuntimeControl_GetPayloadLen(&g_runtime);
		tx_period_ms = RuntimeControl_GetTxPeriodMs(&g_runtime);
		if (tx_period_ms < 10U)
		{
			tx_period_ms = 10U;
		}

		if (!RuntimeControl_IsTxEnabled(&g_runtime))
		{
			wait_start_ms = HAL_GetTick();
			while ((HAL_GetTick() - wait_start_ms) < tx_period_ms)
			{
				RuntimeControl_Poll(&g_runtime);
				HAL_Delay(1);
			}
			continue;
		}

		HAL_GPIO_WritePin(LR1121_TX_IND_PORT, LR1121_TX_IND_PIN, GPIO_PIN_SET);

		st = LR1121_SendLoRaPacket(&hlr1121,
											 RuntimeControl_GetPayload(&g_runtime),
											 payload_len,
											 0x00FFFF);
		if (st == HAL_OK)
		{
			uart_log("TX ok len=%u\r\n", (unsigned int)payload_len);
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

		wait_start_ms = HAL_GetTick();
		while ((HAL_GetTick() - wait_start_ms) < tx_period_ms)
		{
			RuntimeControl_Poll(&g_runtime);
			HAL_Delay(1);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	RuntimeControl_OnUartRxCplt(&g_runtime, huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	RuntimeControl_OnUartError(&g_runtime, huart);
}

void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
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
