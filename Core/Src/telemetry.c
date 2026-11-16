#include "telemetry.h"
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart5;

#define TX_BUF_SIZE   1200U
#define SCAN_POINTS   LIDAR_BINS
#define DATA_FIXED    10U
#define STATS_LEN     32U

static uint8_t tx_buf[TX_BUF_SIZE];
static volatile uint8_t tx_busy;
static telemetry_stats_t stats;
static DMA_HandleTypeDef hdma_uart5_tx;

static void telemetry_dma_init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_uart5_tx.Instance = DMA1_Stream7;
  hdma_uart5_tx.Init.Channel = DMA_CHANNEL_4;
  hdma_uart5_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_uart5_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_uart5_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_uart5_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_uart5_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_uart5_tx.Init.Mode = DMA_NORMAL;
  hdma_uart5_tx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_uart5_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_uart5_tx) != HAL_OK)
  {
    stats.errors++;
    return;
  }
  __HAL_LINKDMA(&huart5, hdmatx, hdma_uart5_tx);

  /* Priority 6: this TX-complete ISR does not call any FreeRTOS API, but it
   * must stay below configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5) so that it
   * cannot preempt FreeRTOS critical sections. */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
}

void DMA1_Stream7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_uart5_tx);
}


static void put16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v & 0xFFU);
  p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)(v & 0xFFU);
  p[1] = (uint8_t)((v >> 8) & 0xFFU);
  p[2] = (uint8_t)((v >> 16) & 0xFFU);
  p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static int16_t sat16(int32_t v)
{
  if (v > 32767)
  {
    return 32767;
  }
  if (v < -32768)
  {
    return -32768;
  }
  return (int16_t)v;
}

static int16_t to_centi(float v)
{
  return sat16((int32_t)((v * 100.0f) + ((v >= 0.0f) ? 0.5f : -0.5f)));
}

static int16_t to_milli(float v)
{
  return sat16((int32_t)((v * 1000.0f) + ((v >= 0.0f) ? 0.5f : -0.5f)));
}

static uint16_t frame_start(uint8_t type, uint16_t payload_len)
{
  tx_buf[0] = TELEMETRY_SYNC0;
  tx_buf[1] = TELEMETRY_SYNC1;
  tx_buf[2] = type;
  tx_buf[3] = (uint8_t)(payload_len & 0xFFU);
  tx_buf[4] = (uint8_t)(payload_len >> 8);
  return 5U;
}

static uint16_t frame_finish(uint16_t pos)
{
  uint8_t x = 0U;
  for (uint16_t i = 2U; i < pos; i++)
  {
    x ^= tx_buf[i];
  }
  tx_buf[pos] = x;
  return (uint16_t)(pos + 1U);
}

static int frame_send(uint16_t len)
{
  tx_busy = 1U;
  if (HAL_UART_Transmit_DMA(&huart5, tx_buf, len) != HAL_OK)
  {
    tx_busy = 0U;
    stats.errors++;
    return -1;
  }
  stats.frames++;
  stats.bytes += len;
  return 0;
}

static uint32_t bt_found_baud;

static void bt_set_baud(uint32_t baud)
{
  huart5.Init.BaudRate = baud;
  HAL_UART_Init(&huart5);
}

static int bt_command(const char *cmd, uint32_t wait_ms)
{
  uint8_t rx[32];
  uint16_t n = 0U;
  uint8_t ch;
  uint32_t t0;

  __HAL_UART_CLEAR_OREFLAG(&huart5);
  while (HAL_UART_Receive(&huart5, &ch, 1U, 0U) == HAL_OK)
  {
  }
  HAL_UART_Transmit(&huart5, (uint8_t *)cmd, (uint16_t)strlen(cmd), 50U);
  t0 = HAL_GetTick();
  while (((HAL_GetTick() - t0) < wait_ms) && (n < (sizeof(rx) - 1U)))
  {
    if (HAL_UART_Receive(&huart5, &ch, 1U, 5U) == HAL_OK)
    {
      rx[n++] = ch;
    }
  }
  rx[n] = 0U;
  return (strstr((const char *)rx, "OK") != NULL) ? 1 : 0;
}

static void bt_autobaud(void)
{
  static const uint32_t bauds[] = {230400U, 9600U, 115200U, 38400U, 57600U, 19200U, 460800U, 921600U};
  uint32_t i;

  bt_found_baud = 0U;
  for (i = 0U; i < (sizeof(bauds) / sizeof(bauds[0])); i++)
  {
    bt_set_baud(bauds[i]);
    if (bt_command("AT\r\n", 200U) != 0)
    {
      bt_found_baud = bauds[i];
      break;
    }
  }

  if ((bt_found_baud != 0U) && (bt_found_baud != 230400U))
  {
    bt_command("AT+BAUD=230400,NONE\r\n", 300U);
    bt_set_baud(230400U);
    HAL_Delay(200U);
    if (bt_command("AT\r\n", 200U) == 0)
    {
      bt_set_baud(bt_found_baud);
      return;
    }
  }
  bt_set_baud(230400U);
}

uint32_t telemetry_bt_baud(void)
{
  return bt_found_baud;
}

void telemetry_init(void)
{
  memset(&stats, 0, sizeof(stats));
  tx_busy = 0U;
  bt_autobaud();
  telemetry_dma_init();
}

int telemetry_send_data(const imu_sample_t *s, const uint16_t *dist)
{
  uint16_t payload_len = (uint16_t)(DATA_FIXED + (SCAN_POINTS * 2U));
  uint16_t pos;

  if (tx_busy != 0U)
  {
    return -1;
  }

  pos = frame_start(TELEMETRY_DATA, payload_len);
  put16(&tx_buf[pos], (uint16_t)to_centi(s->heading_deg));
  pos += 2U;
  put16(&tx_buf[pos], (uint16_t)to_centi(s->gz_dps));
  pos += 2U;
  put16(&tx_buf[pos], (uint16_t)to_milli(s->ax_g));
  pos += 2U;
  put16(&tx_buf[pos], (uint16_t)to_milli(s->ay_g));
  pos += 2U;
  put16(&tx_buf[pos], (uint16_t)SCAN_POINTS);
  pos += 2U;

  for (uint16_t i = 0U; i < SCAN_POINTS; i++)
  {
    put16(&tx_buf[pos], dist[i]);
    pos += 2U;
  }

  pos = frame_finish(pos);
  return frame_send(pos);
}

int telemetry_send_stats(uint32_t frames_sent, uint32_t bytes_per_s,
                         const lidar_stats_t *ls, const imu_stats_t *is)
{
  uint16_t pos;

  if (tx_busy != 0U)
  {
    return -1;
  }

  pos = frame_start(TELEMETRY_STATS, STATS_LEN);
  put32(&tx_buf[pos], frames_sent);
  pos += 4U;
  put32(&tx_buf[pos], bytes_per_s);
  pos += 4U;
  put32(&tx_buf[pos], ls->packets);
  pos += 4U;
  put32(&tx_buf[pos], ls->invalid);
  pos += 4U;
  put32(&tx_buf[pos], ls->proc_us_max);
  pos += 4U;
  put16(&tx_buf[pos], (uint16_t)((ls->ht_count > 65535U) ? 65535U : ls->ht_count));
  pos += 2U;
  put16(&tx_buf[pos], (uint16_t)((ls->tc_count > 65535U) ? 65535U : ls->tc_count));
  pos += 2U;
  put16(&tx_buf[pos], (uint16_t)ls->restarts);
  pos += 2U;
  put16(&tx_buf[pos], (uint16_t)ls->uart_errors);
  pos += 2U;
  put16(&tx_buf[pos], (uint16_t)stats.drops);
  pos += 2U;
  put16(&tx_buf[pos], (uint16_t)is->errors);
  pos += 2U;

  pos = frame_finish(pos);
  return frame_send(pos);
}

void telemetry_on_error(void)
{
  HAL_UART_AbortTransmit(&huart5);
  tx_busy = 0U;
  stats.errors++;
}

void telemetry_get_stats(telemetry_stats_t *out)
{
  memcpy(out, &stats, sizeof(stats));
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    tx_busy = 0U;
  }
}
