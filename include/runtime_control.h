#ifndef RUNTIME_CONTROL_H
#define RUNTIME_CONTROL_H

#include "main.h"

#include <stdint.h>

typedef struct
{
  LR1121_HandleTypeDef *radio;
  UART_HandleTypeDef *uart;
  LR1121_LoRaProfile profile;
  uint8_t payload[255];
  uint8_t rx_byte;
  uint8_t rx_fifo[256];
  volatile uint16_t rx_head;
  volatile uint16_t rx_tail;
  char cmd_line[96];
  uint16_t cmd_idx;
  bool tx_enabled;
  uint32_t tx_period_ms;
} RuntimeControlCtx;

void RuntimeControl_Init(RuntimeControlCtx *ctx,
                         LR1121_HandleTypeDef *radio,
                         UART_HandleTypeDef *uart,
                         const LR1121_LoRaProfile *initial_profile);
HAL_StatusTypeDef RuntimeControl_ApplyInitial(RuntimeControlCtx *ctx);
void RuntimeControl_PrintWelcome(const RuntimeControlCtx *ctx);
void RuntimeControl_Poll(RuntimeControlCtx *ctx);
void RuntimeControl_OnUartRxCplt(RuntimeControlCtx *ctx, UART_HandleTypeDef *huart);
void RuntimeControl_OnUartError(RuntimeControlCtx *ctx, UART_HandleTypeDef *huart);
const LR1121_LoRaProfile *RuntimeControl_GetProfile(const RuntimeControlCtx *ctx);
const uint8_t *RuntimeControl_GetPayload(const RuntimeControlCtx *ctx);
uint8_t RuntimeControl_GetPayloadLen(const RuntimeControlCtx *ctx);
bool RuntimeControl_IsTxEnabled(const RuntimeControlCtx *ctx);
uint32_t RuntimeControl_GetTxPeriodMs(const RuntimeControlCtx *ctx);

#endif
