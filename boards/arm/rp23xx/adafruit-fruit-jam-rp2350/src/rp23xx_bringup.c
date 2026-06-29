/****************************************************************************
 * boards/arm/rp23xx/adafruit-fruit-jam-rp2350/src/rp23xx_bringup.c
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

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/stat.h>

#include <nuttx/fs/fs.h>

#include <arch/board/board.h>

#include "rp23xx_pico.h"

#ifdef CONFIG_ARCH_BOARD_COMMON
#include "rp23xx_common_bringup.h"
#endif /* CONFIG_ARCH_BOARD_COMMON */

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_board_bringup_done;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_mount_tmpfs
 ****************************************************************************/

#ifdef CONFIG_FS_TMPFS
static void rp23xx_mount_tmpfs(FAR const char *path)
{
  int ret;

  ret = mkdir(path, 0777);
  if (ret < 0 && errno != EEXIST)
    {
      syslog(LOG_WARNING, "WARNING: mkdir(%s) failed: %d\n", path, errno);
      return;
    }

  ret = nx_mount(NULL, path, "tmpfs", 0, NULL);
  if (ret < 0 && ret != -EBUSY)
    {
      syslog(LOG_WARNING, "WARNING: nx_mount(tmpfs,%s) failed: %d\n",
             path, ret);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_bringup
 ****************************************************************************/

int rp23xx_bringup(uintptr_t arg)
{
  int ret = OK;

  UNUSED(arg);

  if (g_board_bringup_done)
    {
      return OK;
    }

#ifdef CONFIG_ARCH_BOARD_COMMON

  ret = rp23xx_common_bringup();
  if (ret < 0)
    {
      return ret;
    }

#endif /* CONFIG_ARCH_BOARD_COMMON */

  /* --- Place any board specific bringup code here --- */

#ifdef CONFIG_FS_TMPFS
  rp23xx_mount_tmpfs("/tmp");
  rp23xx_mount_tmpfs("/scripts");
  rp23xx_mount_tmpfs("/data");
#endif

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, \
      "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

  g_board_bringup_done = true;

  return OK;
}
