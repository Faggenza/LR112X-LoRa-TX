#include "lr1121.h"
#include <string.h>

#define LR1121_BUSY_TIMEOUT_PRE_CMD_MS 500U
#define LR1121_BUSY_TIMEOUT_POST_CMD_MS 1500U
#define LR1121_BUSY_TIMEOUT_INIT_MS 500U

static LR1121_DebugInfo g_lr1121_dbg = {
    .stage = LR1121_DBG_NONE,
    .status = HAL_OK,
    .opcode = 0,
    .irq = 0,
};

static void lr1121_set_debug(LR1121_DebugStage stage, HAL_StatusTypeDef status, uint16_t opcode, uint32_t irq)
{
  g_lr1121_dbg.stage = stage;
  g_lr1121_dbg.status = status;
  g_lr1121_dbg.opcode = opcode;
  g_lr1121_dbg.irq = irq;
}

static HAL_StatusTypeDef lr1121_check_error_irq(LR1121_HandleTypeDef *dev,
                                                LR1121_DebugStage stage,
                                                uint16_t opcode)
{
  uint32_t irq = 0U;
  HAL_StatusTypeDef st = LR1121_GetIrqStatus(dev, &irq);

  if (st != HAL_OK)
  {
    lr1121_set_debug(stage, st, opcode, 0U);
    return st;
  }

  if ((irq & (LR1121_IRQ_CMD_ERROR | LR1121_IRQ_ERROR)) != 0U)
  {
    (void)LR1121_ClearIrqStatus(dev, irq);
    lr1121_set_debug(stage, HAL_ERROR, opcode, irq);
    return HAL_ERROR;
  }

  return HAL_OK;
}

static void lr1121_select(LR1121_HandleTypeDef *dev)
{
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void lr1121_deselect(LR1121_HandleTypeDef *dev)
{
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static uint32_t lr1121_rtc_steps_to_ms(uint32_t rtc_steps)
{
  if (rtc_steps == 0U)
  {
    return 5000U;
  }

  return (uint32_t)(((uint64_t)rtc_steps * 1000ULL) / 32768ULL) + 100U;
}

void LR1121_Reset(LR1121_HandleTypeDef *dev)
{
  HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, GPIO_PIN_RESET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, GPIO_PIN_SET);
  HAL_Delay(10);
}

bool LR1121_WaitWhileBusy(LR1121_HandleTypeDef *dev, uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while (HAL_GPIO_ReadPin(dev->busy_port, dev->busy_pin) == GPIO_PIN_SET)
  {
    if ((HAL_GetTick() - start) > timeout_ms)
    {
      return false;
    }
  }

  return true;
}

HAL_StatusTypeDef LR1121_WriteCommand(LR1121_HandleTypeDef *dev,
                                      uint16_t opcode,
                                      const uint8_t *payload,
                                      uint16_t length)
{
  HAL_StatusTypeDef status;
  uint8_t cmd[2] = {(uint8_t)(opcode >> 8), (uint8_t)(opcode & 0xFFU)};

  if (dev == NULL)
  {
    return HAL_ERROR;
  }

  if (!LR1121_WaitWhileBusy(dev, LR1121_BUSY_TIMEOUT_PRE_CMD_MS))
  {
    return HAL_TIMEOUT;
  }

  lr1121_select(dev);
  status = HAL_SPI_Transmit(dev->hspi, cmd, sizeof(cmd), 100);
  if ((status == HAL_OK) && (payload != NULL) && (length > 0U))
  {
    status = HAL_SPI_Transmit(dev->hspi, (uint8_t *)payload, length, 200);
  }
  lr1121_deselect(dev);

  if (status != HAL_OK)
  {
    return status;
  }

  if (!LR1121_WaitWhileBusy(dev, LR1121_BUSY_TIMEOUT_POST_CMD_MS))
  {
    return HAL_TIMEOUT;
  }

  return HAL_OK;
}

HAL_StatusTypeDef LR1121_ReadCommand(LR1121_HandleTypeDef *dev,
                                     uint16_t opcode,
                                     uint8_t *response,
                                     uint16_t length)
{
  HAL_StatusTypeDef status;
  uint8_t cmd[2] = {(uint8_t)(opcode >> 8), (uint8_t)(opcode & 0xFFU)};
  uint8_t tx = 0x00;
  uint8_t dummy = 0x00;

  if ((dev == NULL) || (response == NULL))
  {
    return HAL_ERROR;
  }

  if (!LR1121_WaitWhileBusy(dev, LR1121_BUSY_TIMEOUT_PRE_CMD_MS))
  {
    return HAL_TIMEOUT;
  }

  /* Step 1: write opcode */
  lr1121_select(dev);
  status = HAL_SPI_Transmit(dev->hspi, cmd, sizeof(cmd), 100);
  lr1121_deselect(dev);

  if (status != HAL_OK)
  {
    return status;
  }

  if (!LR1121_WaitWhileBusy(dev, LR1121_BUSY_TIMEOUT_POST_CMD_MS))
  {
    return HAL_TIMEOUT;
  }

  /* Step 2: read and discard one dummy status byte, then read response bytes */
  lr1121_select(dev);
  status = HAL_SPI_TransmitReceive(dev->hspi, &tx, &dummy, 1, 100);

  for (uint16_t i = 0; (i < length) && (status == HAL_OK); i++)
  {
    status = HAL_SPI_TransmitReceive(dev->hspi, &tx, &response[i], 1, 100);
  }
  lr1121_deselect(dev);

  if (status != HAL_OK)
  {
    return status;
  }

  if (!LR1121_WaitWhileBusy(dev, LR1121_BUSY_TIMEOUT_POST_CMD_MS))
  {
    return HAL_TIMEOUT;
  }

  return HAL_OK;
}

HAL_StatusTypeDef LR1121_Init(LR1121_HandleTypeDef *dev)
{
  HAL_StatusTypeDef status;

  if (dev == NULL)
  {
    return HAL_ERROR;
  }

  LR1121_Reset(dev);

  if (!LR1121_WaitWhileBusy(dev, LR1121_BUSY_TIMEOUT_INIT_MS))
  {
    return HAL_TIMEOUT;
  }

  status = LR1121_ClearIrqStatus(dev, LR1121_IRQ_ALL_MASK);
  if (status != HAL_OK)
  {
    return status;
  }

  status = LR1121_ClearErrors(dev);
  if (status != HAL_OK)
  {
    return status;
  }

  return LR1121_SetStandby(dev, LR1121_STDBY_RC);
}

HAL_StatusTypeDef LR1121_SetStandby(LR1121_HandleTypeDef *dev, uint8_t standby_cfg)
{
  return LR1121_WriteCommand(dev, LR1121_CMD_SYSTEM_SET_STANDBY, &standby_cfg, 1);
}

HAL_StatusTypeDef LR1121_SetSleep(LR1121_HandleTypeDef *dev, uint8_t sleep_cfg, uint32_t sleep_time)
{
  uint8_t payload[5];

  payload[0] = sleep_cfg;
  payload[1] = (uint8_t)(sleep_time >> 24);
  payload[2] = (uint8_t)(sleep_time >> 16);
  payload[3] = (uint8_t)(sleep_time >> 8);
  payload[4] = (uint8_t)(sleep_time >> 0);

  return LR1121_WriteCommand(dev, LR1121_CMD_SYSTEM_SET_SLEEP, payload, sizeof(payload));
}

HAL_StatusTypeDef LR1121_SetFs(LR1121_HandleTypeDef *dev)
{
  return LR1121_WriteCommand(dev, LR1121_CMD_SYSTEM_SET_FS, NULL, 0);
}

HAL_StatusTypeDef LR1121_SetTcxoMode(LR1121_HandleTypeDef *dev, uint8_t tune, uint32_t timeout)
{
  uint8_t payload[4];

  payload[0] = tune;
  payload[1] = (uint8_t)(timeout >> 16);
  payload[2] = (uint8_t)(timeout >> 8);
  payload[3] = (uint8_t)(timeout >> 0);

  return LR1121_WriteCommand(dev, LR1121_CMD_SYSTEM_SET_TCXO_MODE, payload, sizeof(payload));
}

HAL_StatusTypeDef LR1121_Calibrate(LR1121_HandleTypeDef *dev, uint8_t calib_mask)
{
  return LR1121_WriteCommand(dev, LR1121_CMD_SYSTEM_CALIBRATE, &calib_mask, 1);
}

HAL_StatusTypeDef LR1121_CalibrateImage(LR1121_HandleTypeDef *dev, uint8_t freq1, uint8_t freq2)
{
  uint8_t payload[2] = {freq1, freq2};
  return LR1121_WriteCommand(dev, LR1121_CMD_SYSTEM_CALIBRATE_IMAGE, payload, sizeof(payload));
}

HAL_StatusTypeDef LR1121_SetRegMode(LR1121_HandleTypeDef *dev, uint8_t reg_mode)
{
  return LR1121_WriteCommand(dev, LR1121_CMD_SYSTEM_SET_REGMODE, &reg_mode, 1);
}

HAL_StatusTypeDef LR1121_SetDioAsRfSwitch(LR1121_HandleTypeDef *dev, const LR1121_RfSwitchCfg *cfg)
{
  uint8_t payload[8];

  if (cfg == NULL)
  {
    return HAL_ERROR;
  }

  payload[0] = cfg->enable;
  payload[1] = cfg->standby;
  payload[2] = cfg->rx;
  payload[3] = cfg->tx;
  payload[4] = cfg->tx_hp;
  payload[5] = cfg->tx_hf;
  payload[6] = cfg->gnss;
  payload[7] = cfg->wifi;

  return LR1121_WriteCommand(dev, LR1121_CMD_SYSTEM_SET_DIO_AS_RF_SWITCH, payload, sizeof(payload));
}

HAL_StatusTypeDef LR1121_SetDioIrqParams(LR1121_HandleTypeDef *dev, uint32_t dio1_mask, uint32_t dio2_mask)
{
  uint8_t payload[8];

  payload[0] = (uint8_t)(dio1_mask >> 24);
  payload[1] = (uint8_t)(dio1_mask >> 16);
  payload[2] = (uint8_t)(dio1_mask >> 8);
  payload[3] = (uint8_t)(dio1_mask >> 0);
  payload[4] = (uint8_t)(dio2_mask >> 24);
  payload[5] = (uint8_t)(dio2_mask >> 16);
  payload[6] = (uint8_t)(dio2_mask >> 8);
  payload[7] = (uint8_t)(dio2_mask >> 0);

  return LR1121_WriteCommand(dev, LR1121_CMD_SYSTEM_SET_DIOIRQPARAMS, payload, sizeof(payload));
}

HAL_StatusTypeDef LR1121_ClearIrqStatus(LR1121_HandleTypeDef *dev, uint32_t mask)
{
  uint8_t payload[4];

  payload[0] = (uint8_t)(mask >> 24);
  payload[1] = (uint8_t)(mask >> 16);
  payload[2] = (uint8_t)(mask >> 8);
  payload[3] = (uint8_t)(mask >> 0);

  return LR1121_WriteCommand(dev, LR1121_CMD_SYSTEM_CLEAR_IRQ, payload, sizeof(payload));
}

HAL_StatusTypeDef LR1121_GetIrqStatus(LR1121_HandleTypeDef *dev, uint32_t *mask)
{
  uint8_t response[4] = {0};
  HAL_StatusTypeDef status;

  if (mask == NULL)
  {
    return HAL_ERROR;
  }

  status = LR1121_ReadCommand(dev, LR1121_CMD_SYSTEM_GET_IRQ_STATUS, response, sizeof(response));
  if (status != HAL_OK)
  {
    return status;
  }

  *mask = ((uint32_t)response[0] << 24) |
          ((uint32_t)response[1] << 16) |
          ((uint32_t)response[2] << 8) |
          ((uint32_t)response[3] << 0);

  return HAL_OK;
}

HAL_StatusTypeDef LR1121_WaitForDio1Irq(LR1121_HandleTypeDef *dev, uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while (HAL_GPIO_ReadPin(dev->dio1_port, dev->dio1_pin) == GPIO_PIN_RESET)
  {
    if ((HAL_GetTick() - start) > timeout_ms)
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef LR1121_GetStatus(LR1121_HandleTypeDef *dev, uint8_t *status)
{
  return LR1121_ReadCommand(dev, LR1121_CMD_SYSTEM_GET_STATUS, status, 1);
}

HAL_StatusTypeDef LR1121_GetErrors(LR1121_HandleTypeDef *dev, uint16_t *errors)
{
  uint8_t response[2] = {0};
  HAL_StatusTypeDef status;

  if (errors == NULL)
  {
    return HAL_ERROR;
  }

  status = LR1121_ReadCommand(dev, LR1121_CMD_SYSTEM_GET_ERRORS, response, sizeof(response));
  if (status != HAL_OK)
  {
    return status;
  }

  *errors = (uint16_t)(((uint16_t)response[0] << 8) | (uint16_t)response[1]);
  return HAL_OK;
}

HAL_StatusTypeDef LR1121_ClearErrors(LR1121_HandleTypeDef *dev)
{
  return LR1121_WriteCommand(dev, LR1121_CMD_SYSTEM_CLEAR_ERRORS, NULL, 0);
}

HAL_StatusTypeDef LR1121_WriteBuffer(LR1121_HandleTypeDef *dev, const uint8_t *data, uint16_t length)
{
  if ((data == NULL) || (length == 0U) || (length > 255U))
  {
    return HAL_ERROR;
  }

  return LR1121_WriteCommand(dev, LR1121_CMD_REGMEM_WRITE_BUFFER8, data, length);
}

HAL_StatusTypeDef LR1121_ReadBuffer(LR1121_HandleTypeDef *dev, uint8_t offset, uint8_t *data, uint16_t length)
{
  uint8_t req[2] = {offset, (uint8_t)length};
  HAL_StatusTypeDef status;

  status = LR1121_WriteCommand(dev, LR1121_CMD_REGMEM_READ_BUFFER8, req, sizeof(req));
  if (status != HAL_OK)
  {
    return status;
  }

  return LR1121_ReadCommand(dev, LR1121_CMD_REGMEM_READ_BUFFER8, data, length);
}

HAL_StatusTypeDef LR1121_WriteRegMem32Mask(LR1121_HandleTypeDef *dev,
                                                uint32_t address,
                                                uint32_t mask,
                                                uint32_t data)
{
  uint8_t payload[12];

  payload[0] = (uint8_t)(address >> 24);
  payload[1] = (uint8_t)(address >> 16);
  payload[2] = (uint8_t)(address >> 8);
  payload[3] = (uint8_t)(address >> 0);
  payload[4] = (uint8_t)(mask >> 24);
  payload[5] = (uint8_t)(mask >> 16);
  payload[6] = (uint8_t)(mask >> 8);
  payload[7] = (uint8_t)(mask >> 0);
  payload[8] = (uint8_t)(data >> 24);
  payload[9] = (uint8_t)(data >> 16);
  payload[10] = (uint8_t)(data >> 8);
  payload[11] = (uint8_t)(data >> 0);

  return LR1121_WriteCommand(dev, LR1121_CMD_REGMEM_WRITE_REGMEM32_MASK, payload, sizeof(payload));
}

HAL_StatusTypeDef LR1121_ApplyHighAcpWorkaround(LR1121_HandleTypeDef *dev)
{
  return LR1121_WriteRegMem32Mask(dev, 0x00F30054UL, (1UL << 30), 0UL);
}

HAL_StatusTypeDef LR1121_SetRfFrequency(LR1121_HandleTypeDef *dev, uint32_t freq_hz)
{
  uint8_t payload[4];

  payload[0] = (uint8_t)(freq_hz >> 24);
  payload[1] = (uint8_t)(freq_hz >> 16);
  payload[2] = (uint8_t)(freq_hz >> 8);
  payload[3] = (uint8_t)(freq_hz >> 0);

  return LR1121_WriteCommand(dev, LR1121_CMD_RADIO_SET_RF_FREQUENCY, payload, sizeof(payload));
}

HAL_StatusTypeDef LR1121_SetPacketType(LR1121_HandleTypeDef *dev, uint8_t packet_type)
{
  return LR1121_WriteCommand(dev, LR1121_CMD_RADIO_SET_PKT_TYPE, &packet_type, 1);
}

HAL_StatusTypeDef LR1121_SetLoRaPublicNetwork(LR1121_HandleTypeDef *dev, uint8_t network_type)
{
  return LR1121_WriteCommand(dev, LR1121_CMD_RADIO_SET_LORA_PUBLIC_NETWORK, &network_type, 1);
}

HAL_StatusTypeDef LR1121_SetModulationParamsLoRa(LR1121_HandleTypeDef *dev,
                                                     uint8_t sf,
                                                     uint8_t bw,
                                                     uint8_t cr,
                                                     uint8_t ldro)
{
  uint8_t payload[4] = {sf, bw, cr, ldro};
  return LR1121_WriteCommand(dev, LR1121_CMD_RADIO_SET_MODULATION_PARAM, payload, sizeof(payload));
}

HAL_StatusTypeDef LR1121_SetPacketParamsLoRa(LR1121_HandleTypeDef *dev,
                                                 uint16_t preamble_len,
                                                 uint8_t header_type,
                                                 uint8_t payload_len,
                                                 uint8_t crc_mode,
                                                 uint8_t invert_iq)
{
  uint8_t payload[6];

  payload[0] = (uint8_t)(preamble_len >> 8);
  payload[1] = (uint8_t)(preamble_len >> 0);
  payload[2] = header_type;
  payload[3] = payload_len;
  payload[4] = crc_mode;
  payload[5] = invert_iq;

  return LR1121_WriteCommand(dev, LR1121_CMD_RADIO_SET_PKT_PARAM, payload, sizeof(payload));
}

HAL_StatusTypeDef LR1121_SetTxParams(LR1121_HandleTypeDef *dev, int8_t power_dbm, uint8_t ramp_time)
{
  uint8_t payload[2] = {(uint8_t)power_dbm, ramp_time};
  return LR1121_WriteCommand(dev, LR1121_CMD_RADIO_SET_TX_PARAMS, payload, sizeof(payload));
}

HAL_StatusTypeDef LR1121_SetPaConfig(LR1121_HandleTypeDef *dev,
                                         LR1121_PaSel pa_sel,
                                         LR1121_PaRegSupply pa_reg_supply,
                                         uint8_t pa_duty_cycle,
                                         uint8_t pa_hp_sel)
{
  uint8_t payload[4];

  payload[0] = (uint8_t)pa_sel;
  payload[1] = (uint8_t)pa_reg_supply;
  payload[2] = pa_duty_cycle;
  payload[3] = pa_hp_sel;

  return LR1121_WriteCommand(dev, LR1121_CMD_RADIO_SET_PA_CFG, payload, sizeof(payload));
}

HAL_StatusTypeDef LR1121_SetTx(LR1121_HandleTypeDef *dev, uint32_t timeout_rtc_step)
{
  uint8_t payload[3];

  payload[0] = (uint8_t)(timeout_rtc_step >> 16);
  payload[1] = (uint8_t)(timeout_rtc_step >> 8);
  payload[2] = (uint8_t)(timeout_rtc_step >> 0);

  return LR1121_WriteCommand(dev, LR1121_CMD_RADIO_SET_TX, payload, sizeof(payload));
}

HAL_StatusTypeDef LR1121_ConfigureLoRa(LR1121_HandleTypeDef *dev, const LR1121_LoRaProfile *profile)
{
  HAL_StatusTypeDef status;
  uint16_t sys_errors = 0U;
  uint32_t freq_mhz;
  uint8_t img_freq1;
  uint8_t img_freq2;
  LR1121_PaSel pa_sel;

  if ((dev == NULL) || (profile == NULL))
  {
    return HAL_ERROR;
  }

  lr1121_set_debug(LR1121_DBG_NONE, HAL_OK, 0, 0U);

  status = LR1121_ClearIrqStatus(dev, LR1121_IRQ_ALL_MASK);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_CFG_CLEAR_IRQ, status, LR1121_CMD_SYSTEM_CLEAR_IRQ, 0U);
    return status;
  }

  status = LR1121_ClearErrors(dev);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_CFG_CLEAR_IRQ, status, LR1121_CMD_SYSTEM_CLEAR_ERRORS, 0U);
    return status;
  }

  status = LR1121_SetStandby(dev, LR1121_STDBY_RC);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_CFG_STANDBY_RC, status, LR1121_CMD_SYSTEM_SET_STANDBY, 0U);
    return status;
  }

  status = LR1121_ClearIrqStatus(dev, LR1121_IRQ_ALL_MASK);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_CFG_CLEAR_IRQ, status, LR1121_CMD_SYSTEM_CLEAR_IRQ, 0U);
    return status;
  }

  status = LR1121_SetDioIrqParams(dev, LR1121_IRQ_TX_DONE | LR1121_IRQ_TIMEOUT | LR1121_IRQ_CMD_ERROR | LR1121_IRQ_ERROR, 0U);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_CFG_SET_DIO_IRQ, status, LR1121_CMD_SYSTEM_SET_DIOIRQPARAMS, 0U);
    return status;
  }

  status = lr1121_check_error_irq(dev, LR1121_DBG_CFG_SET_DIO_IRQ, LR1121_CMD_SYSTEM_SET_DIOIRQPARAMS);
  if (status != HAL_OK)
  {
    return status;
  }

  status = LR1121_Calibrate(dev, 0x3FU);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_CFG_CALIBRATE, status, LR1121_CMD_SYSTEM_CALIBRATE, 0U);
    return status;
  }

  status = LR1121_GetErrors(dev, &sys_errors);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_CFG_CALIBRATE, status, LR1121_CMD_SYSTEM_GET_ERRORS, 0U);
    return status;
  }

  /* Some boards don't have a usable HF XOSC/TCXO path during early bring-up. */
  if ((sys_errors & LR1121_SYS_ERR_HF_XOSC_START) != 0U)
  {
    status = LR1121_SetTcxoMode(dev, LR1121_TCXO_CTRL_3_0V, 0x000200UL);
    if (status != HAL_OK)
    {
      lr1121_set_debug(LR1121_DBG_CFG_SET_TCXO, status, LR1121_CMD_SYSTEM_SET_TCXO_MODE, 0U);
      return status;
    }

    status = LR1121_ClearErrors(dev);
    if (status != HAL_OK)
    {
      lr1121_set_debug(LR1121_DBG_CFG_CLEAR_IRQ, status, LR1121_CMD_SYSTEM_CLEAR_ERRORS, 0U);
      return status;
    }

    status = LR1121_ClearIrqStatus(dev, LR1121_IRQ_ALL_MASK);
    if (status != HAL_OK)
    {
      lr1121_set_debug(LR1121_DBG_CFG_CLEAR_IRQ, status, LR1121_CMD_SYSTEM_CLEAR_IRQ, 0U);
      return status;
    }

    status = LR1121_Calibrate(dev, 0x3FU);
    if (status != HAL_OK)
    {
      lr1121_set_debug(LR1121_DBG_CFG_CALIBRATE, status, LR1121_CMD_SYSTEM_CALIBRATE, 0U);
      return status;
    }

    status = LR1121_GetErrors(dev, &sys_errors);
    if (status != HAL_OK)
    {
      lr1121_set_debug(LR1121_DBG_CFG_CALIBRATE, status, LR1121_CMD_SYSTEM_GET_ERRORS, 0U);
      return status;
    }
  }
  else
  {
    status = lr1121_check_error_irq(dev, LR1121_DBG_CFG_CALIBRATE, LR1121_CMD_SYSTEM_CALIBRATE);
    if (status != HAL_OK)
    {
      return status;
    }
  }

  status = LR1121_SetPacketType(dev, LR1121_PKT_TYPE_LORA);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_CFG_SET_PKT_TYPE, status, LR1121_CMD_RADIO_SET_PKT_TYPE, 0U);
    return status;
  }

  status = lr1121_check_error_irq(dev, LR1121_DBG_CFG_SET_PKT_TYPE, LR1121_CMD_RADIO_SET_PKT_TYPE);
  if (status != HAL_OK)
  {
    return status;
  }

  status = LR1121_SetLoRaPublicNetwork(dev, LR1121_LORA_NETWORK_PRIVATE);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_CFG_SET_PKT_TYPE, status, LR1121_CMD_RADIO_SET_LORA_PUBLIC_NETWORK, 0U);
    return status;
  }

  status = lr1121_check_error_irq(dev, LR1121_DBG_CFG_SET_PKT_TYPE, LR1121_CMD_RADIO_SET_LORA_PUBLIC_NETWORK);
  if (status != HAL_OK)
  {
    return status;
  }

  status = LR1121_SetRfFrequency(dev, profile->frequency_hz);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_CFG_SET_RF_FREQ, status, LR1121_CMD_RADIO_SET_RF_FREQUENCY, 0U);
    return status;
  }

  status = lr1121_check_error_irq(dev, LR1121_DBG_CFG_SET_RF_FREQ, LR1121_CMD_RADIO_SET_RF_FREQUENCY);
  if (status != HAL_OK)
  {
    return status;
  }

  freq_mhz = profile->frequency_hz / 1000000UL;
  img_freq1 = (uint8_t)(((freq_mhz > 8UL) ? (freq_mhz - 8UL) : freq_mhz) / 4UL);
  img_freq2 = (uint8_t)((freq_mhz + 8UL + 3UL) / 4UL);

  status = LR1121_CalibrateImage(dev, img_freq1, img_freq2);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_CFG_CALIBRATE_IMAGE, status, LR1121_CMD_SYSTEM_CALIBRATE_IMAGE, 0U);
    return status;
  }

  status = LR1121_GetErrors(dev, &sys_errors);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_CFG_CALIBRATE_IMAGE, status, LR1121_CMD_SYSTEM_GET_ERRORS, 0U);
    return status;
  }

  if ((sys_errors & LR1121_SYS_ERR_HF_XOSC_START) != 0U)
  {
    status = LR1121_SetTcxoMode(dev, LR1121_TCXO_CTRL_3_0V, 0x000200UL);
    if (status != HAL_OK)
    {
      lr1121_set_debug(LR1121_DBG_CFG_SET_TCXO, status, LR1121_CMD_SYSTEM_SET_TCXO_MODE, 0U);
      return status;
    }

    status = LR1121_ClearErrors(dev);
    if (status != HAL_OK)
    {
      lr1121_set_debug(LR1121_DBG_CFG_CLEAR_IRQ, status, LR1121_CMD_SYSTEM_CLEAR_ERRORS, 0U);
      return status;
    }

    status = LR1121_ClearIrqStatus(dev, LR1121_IRQ_ALL_MASK);
    if (status != HAL_OK)
    {
      lr1121_set_debug(LR1121_DBG_CFG_CLEAR_IRQ, status, LR1121_CMD_SYSTEM_CLEAR_IRQ, 0U);
      return status;
    }

    status = LR1121_CalibrateImage(dev, img_freq1, img_freq2);
    if (status != HAL_OK)
    {
      lr1121_set_debug(LR1121_DBG_CFG_CALIBRATE_IMAGE, status, LR1121_CMD_SYSTEM_CALIBRATE_IMAGE, 0U);
      return status;
    }

    status = LR1121_GetErrors(dev, &sys_errors);
    if (status != HAL_OK)
    {
      lr1121_set_debug(LR1121_DBG_CFG_CALIBRATE_IMAGE, status, LR1121_CMD_SYSTEM_GET_ERRORS, 0U);
      return status;
    }
  }
  else
  {
    status = lr1121_check_error_irq(dev, LR1121_DBG_CFG_CALIBRATE_IMAGE, LR1121_CMD_SYSTEM_CALIBRATE_IMAGE);
    if (status != HAL_OK)
    {
      return status;
    }
  }

  status = LR1121_SetModulationParamsLoRa(dev,
                                              (uint8_t)profile->modulation.sf,
                                              (uint8_t)profile->modulation.bw,
                                              (uint8_t)profile->modulation.cr,
                                              (uint8_t)profile->modulation.ldro);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_CFG_SET_MOD_PARAMS, status, LR1121_CMD_RADIO_SET_MODULATION_PARAM, 0U);
    return status;
  }

  status = lr1121_check_error_irq(dev, LR1121_DBG_CFG_SET_MOD_PARAMS, LR1121_CMD_RADIO_SET_MODULATION_PARAM);
  if (status != HAL_OK)
  {
    return status;
  }

  status = LR1121_SetPacketParamsLoRa(dev,
                                          profile->packet.preamble_len,
                                          (uint8_t)profile->packet.header_type,
                                          profile->packet.payload_len,
                                          (uint8_t)profile->packet.crc,
                                          (uint8_t)profile->packet.iq);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_CFG_SET_PKT_PARAMS, status, LR1121_CMD_RADIO_SET_PKT_PARAM, 0U);
    return status;
  }

  status = lr1121_check_error_irq(dev, LR1121_DBG_CFG_SET_PKT_PARAMS, LR1121_CMD_RADIO_SET_PKT_PARAM);
  if (status != HAL_OK)
  {
    return status;
  }

  pa_sel = (profile->frequency_hz >= 2000000000UL) ? LR1121_PA_SEL_HF : LR1121_PA_SEL_LP;

  status = LR1121_SetPaConfig(dev,
                                  pa_sel,
                                  LR1121_PA_REG_SUPPLY_VREG,
                                  0x04,
                                  0x00);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_CFG_SET_PA_CFG, status, LR1121_CMD_RADIO_SET_PA_CFG, 0U);
    return status;
  }

  status = lr1121_check_error_irq(dev, LR1121_DBG_CFG_SET_PA_CFG, LR1121_CMD_RADIO_SET_PA_CFG);
  if (status != HAL_OK)
  {
    return status;
  }

  status = LR1121_SetTxParams(dev, profile->tx.power_dbm, (uint8_t)profile->tx.ramp);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_CFG_SET_TX_PARAMS, status, LR1121_CMD_RADIO_SET_TX_PARAMS, 0U);
    return status;
  }

  status = lr1121_check_error_irq(dev, LR1121_DBG_CFG_SET_TX_PARAMS, LR1121_CMD_RADIO_SET_TX_PARAMS);
  if (status != HAL_OK)
  {
    return status;
  }

  lr1121_set_debug(LR1121_DBG_NONE, HAL_OK, 0, 0U);
  return HAL_OK;
}

HAL_StatusTypeDef LR1121_LoadLoRaPayload(LR1121_HandleTypeDef *dev,
                                             const uint8_t *payload,
                                             uint8_t payload_len)
{
  if ((payload == NULL) || (payload_len == 0U))
  {
    return HAL_ERROR;
  }

  return LR1121_WriteBuffer(dev, payload, payload_len);
}

HAL_StatusTypeDef LR1121_SendLoRaPacket(LR1121_HandleTypeDef *dev,
                                            const uint8_t *payload,
                                            uint8_t payload_len,
                                            uint32_t timeout_rtc_step)
{
  HAL_StatusTypeDef status;
  uint32_t irq = 0U;

  status = LR1121_ClearIrqStatus(dev, LR1121_IRQ_ALL_MASK);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_TX_CLEAR_IRQ, status, LR1121_CMD_SYSTEM_CLEAR_IRQ, 0U);
    return status;
  }

  status = LR1121_ClearErrors(dev);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_TX_CLEAR_IRQ, status, LR1121_CMD_SYSTEM_CLEAR_ERRORS, 0U);
    return status;
  }

  status = LR1121_SetStandby(dev, LR1121_STDBY_RC);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_TX_STANDBY_RC, status, LR1121_CMD_SYSTEM_SET_STANDBY, 0U);
    return status;
  }

  status = LR1121_LoadLoRaPayload(dev, payload, payload_len);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_TX_WRITE_BUFFER, status, LR1121_CMD_REGMEM_WRITE_BUFFER8, 0U);
    return status;
  }

  status = lr1121_check_error_irq(dev, LR1121_DBG_TX_WRITE_BUFFER, LR1121_CMD_REGMEM_WRITE_BUFFER8);
  if (status != HAL_OK)
  {
    return status;
  }

  status = LR1121_ClearIrqStatus(dev, LR1121_IRQ_ALL_MASK);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_TX_CLEAR_IRQ, status, LR1121_CMD_SYSTEM_CLEAR_IRQ, 0U);
    return status;
  }

  status = LR1121_ApplyHighAcpWorkaround(dev);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_TX_APPLY_HACP, status, LR1121_CMD_REGMEM_WRITE_REGMEM32_MASK, 0U);
    return status;
  }

  status = lr1121_check_error_irq(dev, LR1121_DBG_TX_APPLY_HACP, LR1121_CMD_REGMEM_WRITE_REGMEM32_MASK);
  if (status != HAL_OK)
  {
    return status;
  }

  status = LR1121_SetTx(dev, timeout_rtc_step);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_TX_SET_TX, status, LR1121_CMD_RADIO_SET_TX, 0U);
    return status;
  }

  status = lr1121_check_error_irq(dev, LR1121_DBG_TX_SET_TX, LR1121_CMD_RADIO_SET_TX);
  if (status != HAL_OK)
  {
    return status;
  }

  status = LR1121_WaitForDio1Irq(dev, lr1121_rtc_steps_to_ms(timeout_rtc_step));
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_TX_WAIT_DIO1, status, LR1121_CMD_RADIO_SET_TX, 0U);
    return status;
  }

  status = LR1121_GetIrqStatus(dev, &irq);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_TX_GET_IRQ, status, LR1121_CMD_SYSTEM_GET_IRQ_STATUS, 0U);
    return status;
  }

  status = LR1121_ClearIrqStatus(dev, irq);
  if (status != HAL_OK)
  {
    lr1121_set_debug(LR1121_DBG_TX_CLEAR_DONE_IRQ, status, LR1121_CMD_SYSTEM_CLEAR_IRQ, irq);
    return status;
  }

  if ((irq & (LR1121_IRQ_CMD_ERROR | LR1121_IRQ_ERROR)) != 0U)
  {
    lr1121_set_debug(LR1121_DBG_TX_CHECK_IRQ, HAL_ERROR, LR1121_CMD_RADIO_SET_TX, irq);
    return HAL_ERROR;
  }

  if ((irq & LR1121_IRQ_TX_DONE) == 0U)
  {
    lr1121_set_debug(LR1121_DBG_TX_CHECK_IRQ, HAL_ERROR, LR1121_CMD_RADIO_SET_TX, irq);
    return HAL_ERROR;
  }

  lr1121_set_debug(LR1121_DBG_NONE, HAL_OK, 0, irq);
  return HAL_OK;
}

const LR1121_DebugInfo *LR1121_GetLastDebugInfo(void)
{
  return &g_lr1121_dbg;
}
