#ifndef MAIN_H
#define MAIN_H

#include "stm32l4xx_hal.h"
#include "lr1121.h"

#define APP_LED_PIN GPIO_PIN_1
#define APP_LED_PORT GPIOC

#define LR1121_TX_IND_PIN GPIO_PIN_1
#define LR1121_TX_IND_PORT GPIOC

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;
extern LR1121_HandleTypeDef hlr1121;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_SPI1_Init(void);
void MX_USART2_UART_Init(void);
void Error_Handler(void);

void uart_log(const char *fmt, ...);

#endif
