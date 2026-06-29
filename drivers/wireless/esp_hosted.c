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

#include <nuttx/debug.h>
#include <nuttx/wireless/esp_hosted.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_hosted_driver_s
{
  struct esp_hosted_config_s config;
  struct esp_hosted_stats_s stats;
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

  /* The first real milestone is ESP-Hosted INIT over SPI full duplex.  Do not
   * register a wlan netdev until that path is implemented and verified.
   */

  g_esp_hosted.stats.reset_count++;
  g_esp_hosted.config.gpio->reset(g_esp_hosted.config.gpio_arg, true);
  g_esp_hosted.config.gpio->reset(g_esp_hosted.config.gpio_arg, false);

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
