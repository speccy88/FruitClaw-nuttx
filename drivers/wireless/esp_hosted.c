/****************************************************************************
 * drivers/wireless/esp_hosted.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <endian.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/debug.h>
#include <nuttx/sched.h>
#include <nuttx/wqueue.h>
#include <nuttx/wireless/esp_hosted.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP_HOSTED_RESET_ASSERT_MS       10
#define ESP_HOSTED_RESET_RELEASE_MS      250
#define ESP_HOSTED_HANDSHAKE_POLL_US     1000
#define ESP_HOSTED_HANDSHAKE_POLLS       100
#define ESP_HOSTED_RX_WORK_DRAIN_LIMIT   8

#define ESP_HOSTED_HEADER_LEN            \
  (sizeof(struct esp_hosted_payload_header_s))

#define ESP_HOSTED_MAX_PAYLOAD           \
  (ESP_HOSTED_SPI_MAX_FRAME - ESP_HOSTED_HEADER_LEN)

#define ESP_HOSTED_PRIV_EVENT_HDR_LEN    2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_hosted_driver_s
{
  struct esp_hosted_config_s config;
  struct esp_hosted_stats_s stats;
  struct work_s rx_work;
  volatile bool dataready_seen;
  uint16_t seq_num;
  uint8_t tx_frame[ESP_HOSTED_SPI_MAX_FRAME];
  uint8_t rx_frame[ESP_HOSTED_SPI_MAX_FRAME];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp_hosted_driver_s g_esp_hosted;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void esp_hosted_rx_work(FAR void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t esp_hosted_checksum(FAR const uint8_t *buf, uint16_t len)
{
  uint16_t checksum = 0;
  uint16_t i;

  for (i = 0; i < len; i++)
    {
      checksum += buf[i];
    }

  return checksum;
}

static uint16_t esp_hosted_frame_checksum(FAR uint8_t *buf, uint16_t len)
{
  FAR struct esp_hosted_payload_header_s *header;
  uint16_t saved;
  uint16_t checksum;

  header = (FAR struct esp_hosted_payload_header_s *)buf;
  saved = header->checksum;
  header->checksum = 0;
  checksum = esp_hosted_checksum(buf, len);
  header->checksum = saved;

  return checksum;
}

static void esp_hosted_build_header(FAR struct esp_hosted_driver_s *priv,
                                    FAR uint8_t *frame,
                                    enum esp_hosted_if_type_e if_type,
                                    uint8_t if_num, uint8_t flags,
                                    FAR const uint8_t *payload,
                                    uint16_t payload_len)
{
  FAR struct esp_hosted_payload_header_s *header;
  uint16_t frame_len;

  DEBUGASSERT(payload_len <= ESP_HOSTED_MAX_PAYLOAD);

  memset(frame, 0, ESP_HOSTED_SPI_MAX_FRAME);

  header = (FAR struct esp_hosted_payload_header_s *)frame;
  ESP_HOSTED_HDR_SET_IF(header, if_type, if_num);
  header->flags = flags;

  if (payload_len > 0)
    {
      memcpy(frame + ESP_HOSTED_HEADER_LEN, payload, payload_len);
      header->len = htole16(payload_len);
      header->offset = htole16(ESP_HOSTED_HEADER_LEN);
      header->seq_num = htole16(priv->seq_num++);

      frame_len = ESP_HOSTED_HEADER_LEN + payload_len;
      header->checksum = htole16(esp_hosted_checksum(frame, frame_len));
      priv->stats.tx_frame_count++;
    }
}

static void esp_hosted_build_dummy(FAR struct esp_hosted_driver_s *priv,
                                   FAR uint8_t *frame)
{
  esp_hosted_build_header(priv, frame, ESP_HOSTED_MAX_IF, 0, 0, NULL, 0);
  priv->stats.tx_dummy_count++;
}

static int esp_hosted_parse_priv_frame(FAR struct esp_hosted_driver_s *priv,
                                       FAR const uint8_t *payload,
                                       uint16_t len)
{
  uint8_t event_type;
  uint8_t event_len;

  priv->stats.rx_priv_count++;

  if (len < ESP_HOSTED_PRIV_EVENT_HDR_LEN)
    {
      return -EINVAL;
    }

  event_type = payload[0];
  event_len = payload[1];

  if (event_len + ESP_HOSTED_PRIV_EVENT_HDR_LEN > len)
    {
      return -EINVAL;
    }

  if (event_type == ESP_HOSTED_PRIV_EVENT_INIT)
    {
      priv->stats.rx_init_event_count++;
      ninfo("ESP-Hosted INIT event received: len=%u\n", event_len);
    }
  else
    {
      ninfo("ESP-Hosted private event received: type=%02x len=%u\n",
            event_type, event_len);
    }

  return OK;
}

static int esp_hosted_parse_rx_frame(FAR struct esp_hosted_driver_s *priv,
                                     FAR uint8_t *frame)
{
  FAR struct esp_hosted_payload_header_s *header;
  FAR uint8_t *payload;
  enum esp_hosted_if_type_e if_type;
  uint16_t len;
  uint16_t offset;
  uint16_t checksum;
  uint16_t expected;
  int ret = OK;

  header = (FAR struct esp_hosted_payload_header_s *)frame;
  if_type = ESP_HOSTED_HDR_IF_TYPE(header);
  len = le16toh(header->len);
  offset = le16toh(header->offset);

  if (len == 0)
    {
      priv->stats.rx_dummy_count++;
      return OK;
    }

  if (len > ESP_HOSTED_MAX_PAYLOAD || offset != ESP_HOSTED_HEADER_LEN ||
      offset + len > ESP_HOSTED_SPI_MAX_FRAME)
    {
      priv->stats.malformed_frame_count++;
      nwarn("ESP-Hosted malformed frame: if=%u len=%u offset=%u\n",
            if_type, len, offset);
      return -EINVAL;
    }

  checksum = le16toh(header->checksum);
  expected = esp_hosted_frame_checksum(frame, offset + len);
  if (checksum != expected)
    {
      priv->stats.checksum_error_count++;
      nwarn("ESP-Hosted checksum mismatch: rx=%04x expected=%04x\n",
            checksum, expected);
      return -EIO;
    }

  priv->stats.rx_frame_count++;
  payload = frame + offset;

  switch (if_type)
    {
      case ESP_HOSTED_STA_IF:
        priv->stats.rx_sta_count++;
        break;

      case ESP_HOSTED_AP_IF:
        priv->stats.rx_ap_count++;
        break;

      case ESP_HOSTED_SERIAL_IF:
        priv->stats.rx_control_count++;
        break;

      case ESP_HOSTED_PRIV_IF:
        ret = esp_hosted_parse_priv_frame(priv, payload, len);
        if (ret < 0)
          {
            priv->stats.malformed_frame_count++;
          }
        break;

      default:
        priv->stats.rx_unknown_count++;
        nwarn("ESP-Hosted unsupported frame: if=%u len=%u\n", if_type, len);
        break;
    }

  return ret;
}

static int esp_hosted_validate_config(
  FAR const struct esp_hosted_config_s *config)
{
  if (config == NULL || config->spi == NULL)
    {
      return -EINVAL;
    }

  if (config->spi_frequency == 0 || config->spi_bits == 0)
    {
      return -EINVAL;
    }

  if (config->gpio == NULL || config->gpio->reset == NULL ||
      config->gpio->handshake_ready == NULL ||
      config->gpio->data_ready == NULL)
    {
      return -EINVAL;
    }

  return OK;
}

static void esp_hosted_schedule_rx_work(FAR struct esp_hosted_driver_s *priv)
{
  int ret;

  if (work_available(&priv->rx_work))
    {
      ret = work_queue(HPWORK, &priv->rx_work, esp_hosted_rx_work, priv, 0);
      if (ret >= 0)
        {
          priv->stats.rx_work_count++;
        }
    }
}

static int esp_hosted_dataready_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct esp_hosted_driver_s *priv = arg;

  UNUSED(irq);
  UNUSED(context);

  priv->dataready_seen = true;
  priv->stats.dataready_irq_count++;
  esp_hosted_schedule_rx_work(priv);

  return OK;
}

static int esp_hosted_wait_handshake(FAR struct esp_hosted_driver_s *priv)
{
  unsigned int retries;

  for (retries = 0; retries < ESP_HOSTED_HANDSHAKE_POLLS; retries++)
    {
      if (priv->config.gpio->handshake_ready(priv->config.gpio_arg))
        {
          return OK;
        }

      nxsched_usleep(ESP_HOSTED_HANDSHAKE_POLL_US);
    }

  priv->stats.control_timeout_count++;
  return -ETIMEDOUT;
}

static void esp_hosted_spi_configure(FAR struct esp_hosted_driver_s *priv)
{
  FAR struct spi_dev_s *spi = priv->config.spi;

  SPI_SETFREQUENCY(spi, priv->config.spi_frequency);
  SPI_SETMODE(spi, priv->config.spi_mode);
  SPI_SETBITS(spi, priv->config.spi_bits);
}

static int esp_hosted_spi_exchange_frame(FAR struct esp_hosted_driver_s *priv,
                                         FAR const void *txbuf,
                                         FAR void *rxbuf)
{
  FAR struct spi_dev_s *spi = priv->config.spi;
  int ret;

  ret = esp_hosted_wait_handshake(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = SPI_LOCK(spi, true);
  if (ret < 0)
    {
      return ret;
    }

  esp_hosted_spi_configure(priv);

  SPI_SELECT(spi, priv->config.spi_devid, true);
  SPI_EXCHANGE(spi, txbuf, rxbuf, ESP_HOSTED_SPI_MAX_FRAME);
  SPI_SELECT(spi, priv->config.spi_devid, false);

  SPI_LOCK(spi, false);

  priv->stats.spi_transaction_count++;
  return OK;
}

static int esp_hosted_exchange_dummy(FAR struct esp_hosted_driver_s *priv)
{
  int ret;

  esp_hosted_build_dummy(priv, priv->tx_frame);
  memset(priv->rx_frame, 0, sizeof(priv->rx_frame));

  ret = esp_hosted_spi_exchange_frame(priv, priv->tx_frame, priv->rx_frame);
  if (ret < 0)
    {
      return ret;
    }

  return esp_hosted_parse_rx_frame(priv, priv->rx_frame);
}

static void esp_hosted_rx_work(FAR void *arg)
{
  FAR struct esp_hosted_driver_s *priv = arg;
  unsigned int i;
  int ret;

  for (i = 0; i < ESP_HOSTED_RX_WORK_DRAIN_LIMIT; i++)
    {
      if (!priv->dataready_seen &&
          !priv->config.gpio->data_ready(priv->config.gpio_arg))
        {
          break;
        }

      priv->dataready_seen = false;

      ret = esp_hosted_exchange_dummy(priv);
      if (ret < 0)
        {
          nwarn("ESP-Hosted RX drain failed: %d\n", ret);
          break;
        }
    }
}

static int esp_hosted_attach_irq(FAR struct esp_hosted_driver_s *priv)
{
  int ret;

  if (priv->config.gpio->attach_dataready == NULL ||
      priv->config.gpio->enable_dataready_irq == NULL)
    {
      return OK;
    }

  ret = priv->config.gpio->attach_dataready(priv->config.gpio_arg,
                                            esp_hosted_dataready_isr,
                                            priv);
  if (ret < 0)
    {
      return ret;
    }

  return priv->config.gpio->enable_dataready_irq(priv->config.gpio_arg,
                                                 true);
}

static void esp_hosted_reset_coprocessor(FAR struct esp_hosted_driver_s *priv)
{
  priv->stats.reset_count++;

  priv->config.gpio->reset(priv->config.gpio_arg, true);
  up_mdelay(ESP_HOSTED_RESET_ASSERT_MS);
  priv->config.gpio->reset(priv->config.gpio_arg, false);
  up_mdelay(ESP_HOSTED_RESET_RELEASE_MS);
}

static void esp_hosted_prime_transport(FAR struct esp_hosted_driver_s *priv)
{
  int ret;

  if (!priv->config.gpio->handshake_ready(priv->config.gpio_arg))
    {
      ninfo("ESP-Hosted handshake is not ready after reset\n");
      return;
    }

  ret = esp_hosted_exchange_dummy(priv);
  if (ret < 0)
    {
      nwarn("ESP-Hosted initial SPI exchange failed: %d\n", ret);
    }
  else
    {
      ninfo("ESP-Hosted initial SPI exchange completed\n");
    }

  if (priv->config.gpio->data_ready(priv->config.gpio_arg))
    {
      priv->dataready_seen = true;
      esp_hosted_schedule_rx_work(priv);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int esp_hosted_spi_initialize(FAR const struct esp_hosted_config_s *config)
{
  int ret;

  ret = esp_hosted_validate_config(config);
  if (ret < 0)
    {
      return ret;
    }

  memset(&g_esp_hosted, 0, sizeof(g_esp_hosted));
  g_esp_hosted.config = *config;

  /* The first real milestone is ESP-Hosted INIT over SPI full duplex. Do not
   * register a wlan netdev until that path is implemented and verified.
   */

  ret = esp_hosted_attach_irq(&g_esp_hosted);
  if (ret < 0)
    {
      return ret;
    }

  esp_hosted_reset_coprocessor(&g_esp_hosted);
  esp_hosted_prime_transport(&g_esp_hosted);

  ninfo("ESP-Hosted SPI scaffold ready: frame=%u checksum=%04" PRIx16 "\n",
        ESP_HOSTED_SPI_MAX_FRAME,
        esp_hosted_checksum((FAR const uint8_t *)ESP_HOSTED_RPC_EP_NAME_RSP,
                            sizeof(ESP_HOSTED_RPC_EP_NAME_RSP) - 1));

  return -ENOSYS;
}

int esp_hosted_spi_get_stats(FAR struct esp_hosted_stats_s *stats)
{
  if (stats == NULL)
    {
      return -EINVAL;
    }

  *stats = g_esp_hosted.stats;
  return OK;
}
