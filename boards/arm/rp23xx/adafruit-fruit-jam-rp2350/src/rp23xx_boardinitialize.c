/****************************************************************************
 * boards/arm/rp23xx/adafruit-fruit-jam-rp2350/src/rp23xx_boardinitialize.c
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

#include <stdbool.h>

#include <nuttx/debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "rp23xx_gpio.h"

#ifdef CONFIG_RP23XX_PSRAM
#include "rp23xx_psram.h"
#endif

#ifdef CONFIG_ARCH_BOARD_COMMON
#include "rp23xx_common_initialize.h"
#endif /* CONFIG_ARCH_BOARD_COMMON */

#include "rp23xx_pico.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ADAFRUIT_FRUIT_JAM_RP2350_BOOT_DIAG
static void fruitjam_bootdiag_led(bool ledon)
{
  rp23xx_gpio_init(GPIO_LED1);
  rp23xx_gpio_setdir(GPIO_LED1, true);
  rp23xx_gpio_put(GPIO_LED1, !ledon);
}
#endif

#ifdef CONFIG_ADAFRUIT_FRUIT_JAM_RP2350_NINA
static void fruitjam_nina_initialize(void)
{
  rp23xx_gpio_init(BOARD_NINA_RESET_PIN);
  rp23xx_gpio_setdir(BOARD_NINA_RESET_PIN, true);
  rp23xx_gpio_put(BOARD_NINA_RESET_PIN, true);

  rp23xx_gpio_init(BOARD_NINA_READY_PIN);
  rp23xx_gpio_setdir(BOARD_NINA_READY_PIN, false);

  rp23xx_gpio_init(BOARD_NINA_IRQ_PIN);
  rp23xx_gpio_setdir(BOARD_NINA_IRQ_PIN, false);

  rp23xx_gpio_init(BOARD_NINA_CS_PIN);
  rp23xx_gpio_setdir(BOARD_NINA_CS_PIN, true);
  rp23xx_gpio_put(BOARD_NINA_CS_PIN, true);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_boardearlyinitialize
 *
 * Description:
 *
 ****************************************************************************/

void rp23xx_boardearlyinitialize(void)
{
  #ifdef CONFIG_ARCH_BOARD_COMMON
  rp23xx_common_earlyinitialize();
  #endif

  /* --- Place any board specific early initialization here --- */

#ifdef CONFIG_ADAFRUIT_FRUIT_JAM_RP2350_BOOT_DIAG
  fruitjam_bootdiag_led(true);
#endif
}

/****************************************************************************
 * Name: rp23xx_boardinitialize
 *
 * Description:
 *
 ****************************************************************************/

void rp23xx_boardinitialize(void)
{
  #ifdef CONFIG_ARCH_BOARD_COMMON
  rp23xx_common_initialize();
  #endif

  #ifdef CONFIG_RP23XX_PSRAM
  rp23xx_psramconfig();
  #endif

  /* --- Place any board specific initialization here --- */

#ifdef CONFIG_ADAFRUIT_FRUIT_JAM_RP2350_NINA
  fruitjam_nina_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize(). board_late_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  rp23xx_bringup(0);
}
#endif
