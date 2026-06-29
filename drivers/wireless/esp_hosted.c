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

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/debug.h>
#include <nuttx/sched.h>
#include <nuttx/wireless/esp_hosted.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP_HOSTED_RESET_ASSERT_MS       10
#define ESP_HOSTED_RESET_RELEASE_MS      250
#define ESP_HOSTED_HANDSHAKE_POLL_US     1000
#define ESP_HOSTED_HANDSHAKE_POLLS       100

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_hosted_driver_s
{
  struct esp_hosted_config_s config;
  struct esp_hosted_stats_s stats;
  volatile bool dataready_seen;
  uint8_t tx_frame[ESP_HOSTED_SPI_MAX_FRAME];
  uint8_t rx_frame[ESP_HOSTED_SPI_MAX_FRAME];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp_hosted_driver_s g_esp_hosted;

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

static int esp_hosted_dataready_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct esp_hosted_driver_s *priv = arg;

  UNUSED(irq);
  UNUSED(context);

  priv->dataready_seen = true;
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

  memset(priv->tx_frame, 0, sizeof(priv->tx_frame));
  memset(priv->rx_frame, 0, sizeof(priv->rx_frame));

  ret = esp_hosted_spi_exchange_frame(priv, priv->tx_frame, priv->rx_frame);
  if (ret < 0)
    {
      nwarn("ESP-Hosted initial SPI exchange failed: %d\n", ret);
    }
  else
    {
      ninfo("ESP-Hosted initial SPI exchange completed\n");
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
