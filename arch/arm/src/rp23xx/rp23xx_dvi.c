/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_dvi.c
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

#include <sys/types.h>
#include <sys/param.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/video/fb.h>

#include <arch/board/board.h>
#include <arch/rp23xx/irq.h>

#include "arm_internal.h"

#include "rp23xx_dmac.h"
#include "rp23xx_gpio.h"

#include "hardware/rp23xx_accessctrl.h"
#include "hardware/rp23xx_dma.h"
#include "hardware/rp23xx_dreq.h"
#include "hardware/rp23xx_hstx_ctrl.h"
#include "hardware/rp23xx_hstx_fifo.h"
#include "hardware/rp23xx_memorymap.h"
#include "hardware/rp23xx_pads_bank0.h"

#ifdef CONFIG_RP23XX_HSTX_DVI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TMDS control symbols */

#define TMDS_CTRL_00                     0x354u
#define TMDS_CTRL_01                     0x0abu
#define TMDS_CTRL_10                     0x154u
#define TMDS_CTRL_11                     0x2abu

#define SYNC_V0_H0                       (TMDS_CTRL_00 | \
                                          (TMDS_CTRL_00 << 10) | \
                                          (TMDS_CTRL_00 << 20))
#define SYNC_V0_H1                       (TMDS_CTRL_01 | \
                                          (TMDS_CTRL_00 << 10) | \
                                          (TMDS_CTRL_00 << 20))
#define SYNC_V1_H0                       (TMDS_CTRL_10 | \
                                          (TMDS_CTRL_00 << 10) | \
                                          (TMDS_CTRL_00 << 20))
#define SYNC_V1_H1                       (TMDS_CTRL_11 | \
                                          (TMDS_CTRL_00 << 10) | \
                                          (TMDS_CTRL_00 << 20))

/* 640x480 @ 60 Hz timing */

#define H_FRONT_PORCH                    16
#define H_SYNC_WIDTH                     96
#define H_BACK_PORCH                     48
#define H_ACTIVE_PIXELS                  640

#define V_FRONT_PORCH                    10
#define V_SYNC_WIDTH                     2
#define V_BACK_PORCH                     33
#define V_ACTIVE_LINES                   480
#define V_TOTAL_LINES                    (V_FRONT_PORCH + V_SYNC_WIDTH + \
                                          V_BACK_PORCH + V_ACTIVE_LINES)

/* HSTX command types */

#define HSTX_CMD_RAW                     (0x0u << 12)
#define HSTX_CMD_RAW_REPEAT              (0x1u << 12)
#define HSTX_CMD_TMDS                    (0x2u << 12)
#define HSTX_CMD_TMDS_REPEAT             (0x3u << 12)
#define HSTX_CMD_NOP                     (0xfu << 12)

/* Logical framebuffer and output scaling */

#define RP23XX_DVI_FB_WIDTH              128
#define RP23XX_DVI_FB_HEIGHT             128
#define RP23XX_DVI_OUTPUT_SCALING        3
#define RP23XX_DVI_BPP                   16

#define H_BORDER                         ((H_ACTIVE_PIXELS - \
                                          RP23XX_DVI_FB_WIDTH * \
                                          RP23XX_DVI_OUTPUT_SCALING) / 2)
#define V_BORDER                         ((V_ACTIVE_LINES - \
                                          RP23XX_DVI_FB_HEIGHT * \
                                          RP23XX_DVI_OUTPUT_SCALING) / 2)
#define RP23XX_DVI_CONTENT_LINES         (RP23XX_DVI_FB_HEIGHT * \
                                          RP23XX_DVI_OUTPUT_SCALING)

#define VSYNC_LEN                        6
#define VACTIVE_PRE_LEN                  11
#define VACTIVE_POST_LEN                 2
#define VACTIVE_BLACK_LEN                10

/* Every scanline needs one control block. Content lines need three. Add one
 * terminating block to generate the end-of-frame DMA interrupt.
 */

#define DMA_CMD_BUF_WORDS                (((V_TOTAL_LINES - \
                                          RP23XX_DVI_CONTENT_LINES) * 4) + \
                                          (RP23XX_DVI_CONTENT_LINES * 12) + \
                                          4)

#define RP23XX_DVI_FRAME_BYTES           (RP23XX_DVI_FB_WIDTH * \
                                          RP23XX_DVI_FB_HEIGHT * 2)

#define RP23XX_DVI_PIXEL_CLOCK           25200000u
#define RP23XX_DVI_TREQL_HSTX            RP23XX_DMA_DREQ_HSTX

#define RP23XX_DVI_BUS_PRIORITY          (RP23XX_BUSCTRL_BASE + 0x00000000)
#define RP23XX_DVI_BUS_PRIORITY_DMA_W    (1u << 12)
#define RP23XX_DVI_BUS_PRIORITY_DMA_R    (1u << 8)

/* HSTX register shifts not currently named by the generated header. */

#define HSTX_CSR_CLKDIV_SHIFT            28
#define HSTX_CSR_N_SHIFTS_SHIFT          16
#define HSTX_CSR_SHIFT_SHIFT             8
#define HSTX_BIT_SEL_N_SHIFT             8
#define HSTX_EXPAND_ENC_N_SHIFTS_SHIFT   24
#define HSTX_EXPAND_ENC_SHIFT_SHIFT      16
#define HSTX_EXPAND_RAW_N_SHIFTS_SHIFT   8
#define HSTX_EXPAND_TMDS_L2_NBITS_SHIFT  21
#define HSTX_EXPAND_TMDS_L2_ROT_SHIFT    16
#define HSTX_EXPAND_TMDS_L1_NBITS_SHIFT  13
#define HSTX_EXPAND_TMDS_L1_ROT_SHIFT    8
#define HSTX_EXPAND_TMDS_L0_NBITS_SHIFT  5

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rp23xx_dvi_s
{
  struct fb_vtable_s vtable;
  DMA_HANDLE pixel_dma;
  DMA_HANDLE command_dma;
  unsigned int pixel_ch;
  unsigned int command_ch;
  int power;
  bool initialized;
  bool streaming;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rp23xx_dvi_getvideoinfo(FAR struct fb_vtable_s *vtable,
                                   FAR struct fb_videoinfo_s *vinfo);
static int rp23xx_dvi_getplaneinfo(FAR struct fb_vtable_s *vtable,
                                   int planeno,
                                   FAR struct fb_planeinfo_s *pinfo);
static int rp23xx_dvi_open(FAR struct fb_vtable_s *vtable);
static int rp23xx_dvi_close(FAR struct fb_vtable_s *vtable);
#ifdef CONFIG_FB_UPDATE
static int rp23xx_dvi_updatearea(FAR struct fb_vtable_s *vtable,
                                 FAR const struct fb_area_s *area);
#endif
static int rp23xx_dvi_setframerate(FAR struct fb_vtable_s *vtable,
                                   int rate);
static int rp23xx_dvi_getframerate(FAR struct fb_vtable_s *vtable);
static int rp23xx_dvi_getpower(FAR struct fb_vtable_s *vtable);
static int rp23xx_dvi_setpower(FAR struct fb_vtable_s *vtable, int power);
static int rp23xx_dvi_ioctl(FAR struct fb_vtable_s *vtable, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_vblank_vsync_off[VSYNC_LEN] =
{
  HSTX_CMD_RAW_REPEAT | H_FRONT_PORCH,
  SYNC_V1_H1,
  HSTX_CMD_RAW_REPEAT | H_SYNC_WIDTH,
  SYNC_V1_H0,
  HSTX_CMD_RAW_REPEAT | (H_BACK_PORCH + H_ACTIVE_PIXELS),
  SYNC_V1_H1
};

static uint32_t g_vblank_vsync_on[VSYNC_LEN] =
{
  HSTX_CMD_RAW_REPEAT | H_FRONT_PORCH,
  SYNC_V0_H1,
  HSTX_CMD_RAW_REPEAT | H_SYNC_WIDTH,
  SYNC_V0_H0,
  HSTX_CMD_RAW_REPEAT | (H_BACK_PORCH + H_ACTIVE_PIXELS),
  SYNC_V0_H1
};

static uint32_t g_vactive_pre[VACTIVE_PRE_LEN] =
{
  HSTX_CMD_RAW_REPEAT | H_FRONT_PORCH,
  SYNC_V1_H1,
  HSTX_CMD_NOP,
  HSTX_CMD_RAW_REPEAT | H_SYNC_WIDTH,
  SYNC_V1_H0,
  HSTX_CMD_NOP,
  HSTX_CMD_RAW_REPEAT | H_BACK_PORCH,
  SYNC_V1_H1,
  HSTX_CMD_TMDS_REPEAT | H_BORDER,
  0x00000000,
  HSTX_CMD_TMDS | (RP23XX_DVI_FB_WIDTH * RP23XX_DVI_OUTPUT_SCALING)
};

static uint32_t g_vactive_post[VACTIVE_POST_LEN] =
{
  HSTX_CMD_TMDS_REPEAT | H_BORDER,
  0x00000000
};

static uint32_t g_vactive_black[VACTIVE_BLACK_LEN] =
{
  HSTX_CMD_RAW_REPEAT | H_FRONT_PORCH,
  SYNC_V1_H1,
  HSTX_CMD_NOP,
  HSTX_CMD_RAW_REPEAT | H_SYNC_WIDTH,
  SYNC_V1_H0,
  HSTX_CMD_NOP,
  HSTX_CMD_RAW_REPEAT | H_BACK_PORCH,
  SYNC_V1_H1,
  HSTX_CMD_TMDS_REPEAT | H_ACTIVE_PIXELS,
  0x00000000
};

static uint32_t g_dma_commands[DMA_CMD_BUF_WORDS];
static uint16_t g_framebuffer[RP23XX_DVI_FB_WIDTH * RP23XX_DVI_FB_HEIGHT];

static struct rp23xx_dvi_s g_dvi =
{
  .vtable =
    {
      .getvideoinfo = rp23xx_dvi_getvideoinfo,
      .getplaneinfo = rp23xx_dvi_getplaneinfo,
      .open         = rp23xx_dvi_open,
      .close        = rp23xx_dvi_close,
#ifdef CONFIG_FB_UPDATE
      .updatearea   = rp23xx_dvi_updatearea,
#endif
      .setframerate = rp23xx_dvi_setframerate,
      .getframerate = rp23xx_dvi_getframerate,
      .getpower     = rp23xx_dvi_getpower,
      .setpower     = rp23xx_dvi_setpower,
      .ioctl        = rp23xx_dvi_ioctl,
    },
  .power = 0,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static unsigned int rp23xx_dvi_dma_channel(DMA_HANDLE handle)
{
  uintptr_t addr = rp23xx_dma_register(handle, RP23XX_DMA_CTRL_TRIG_OFFSET);

  return (addr - RP23XX_DMA_CTRL_TRIG(0)) / 0x40;
}

static void rp23xx_dvi_push_block(FAR uint32_t *cw, uint32_t ctrl,
                                  uint32_t write_addr, uint32_t count,
                                  uintptr_t read_addr)
{
  g_dma_commands[(*cw)++] = ctrl;
  g_dma_commands[(*cw)++] = write_addr;
  g_dma_commands[(*cw)++] = count;
  g_dma_commands[(*cw)++] = read_addr;
}

static void rp23xx_dvi_build_command_list(void)
{
  const uint32_t dma_ctrl_base =
    (g_dvi.command_ch << RP23XX_DMA_CTRL_TRIG_CHAIN_TO_SHIFT) |
    (RP23XX_DVI_TREQL_HSTX << RP23XX_DMA_CTRL_TRIG_TREQ_SEL_SHIFT) |
    RP23XX_DMA_CTRL_TRIG_IRQ_QUIET |
    RP23XX_DMA_CTRL_TRIG_INCR_READ |
    RP23XX_DMA_CTRL_TRIG_HIGH_PRIORITY |
    RP23XX_DMA_CTRL_TRIG_EN;
  const uint32_t dma_pixel_ctrl =
    dma_ctrl_base |
    (RP23XX_DMA_SIZE_HALFWORD << RP23XX_DMA_CTRL_TRIG_DATA_SIZE_SHIFT);
  const uint32_t dma_ctrl =
    dma_ctrl_base |
    (RP23XX_DMA_SIZE_WORD << RP23XX_DMA_CTRL_TRIG_DATA_SIZE_SHIFT);
  const uint32_t dma_write_addr = RP23XX_HSTX_FIFO_FIFO;

  const unsigned int vsync_start = 0;
  const unsigned int vsync_end = V_SYNC_WIDTH;
  const unsigned int backporch_end = vsync_end + V_BACK_PORCH;
  const unsigned int active_start = backporch_end;
  const unsigned int frontporch_start = V_TOTAL_LINES - V_FRONT_PORCH;
  const unsigned int content_start = active_start + V_BORDER;
  const unsigned int content_end =
    content_start + RP23XX_DVI_CONTENT_LINES;
  uint32_t cw = 0;
  unsigned int line;

  for (line = 0; line < V_TOTAL_LINES; line++)
    {
      if (line >= vsync_start && line < vsync_end)
        {
          rp23xx_dvi_push_block(&cw, dma_ctrl, dma_write_addr, VSYNC_LEN,
                                (uintptr_t)g_vblank_vsync_on);
        }
      else if (line >= active_start && line < frontporch_start)
        {
          if (line >= content_start && line < content_end)
            {
              const unsigned int row =
                (line - content_start) / RP23XX_DVI_OUTPUT_SCALING;

              rp23xx_dvi_push_block(&cw, dma_ctrl, dma_write_addr,
                                    VACTIVE_PRE_LEN,
                                    (uintptr_t)g_vactive_pre);
              rp23xx_dvi_push_block(&cw, dma_pixel_ctrl, dma_write_addr,
                                    RP23XX_DVI_FB_WIDTH,
                                    (uintptr_t)&g_framebuffer
                                      [row * RP23XX_DVI_FB_WIDTH]);
              rp23xx_dvi_push_block(&cw, dma_ctrl, dma_write_addr,
                                    VACTIVE_POST_LEN,
                                    (uintptr_t)g_vactive_post);
            }
          else
            {
              rp23xx_dvi_push_block(&cw, dma_ctrl, dma_write_addr,
                                    VACTIVE_BLACK_LEN,
                                    (uintptr_t)g_vactive_black);
            }
        }
      else
        {
          rp23xx_dvi_push_block(&cw, dma_ctrl, dma_write_addr, VSYNC_LEN,
                                (uintptr_t)g_vblank_vsync_off);
        }
    }

  rp23xx_dvi_push_block(&cw,
                        RP23XX_DMA_CTRL_TRIG_IRQ_QUIET |
                        RP23XX_DMA_CTRL_TRIG_EN,
                        0, 0, 0);

  DEBUGASSERT(cw == DMA_CMD_BUF_WORDS);
}

static int rp23xx_dvi_dma_interrupt(int irq, void *context, void *arg)
{
  const uint32_t bit = 1u << g_dvi.pixel_ch;
  const uint32_t errors = RP23XX_DMA_CTRL_TRIG_READ_ERROR |
                          RP23XX_DMA_CTRL_TRIG_WRITE_ERROR;
  uint32_t ctrl;

  putreg32(bit, RP23XX_DMA_INTS2);

  ctrl = getreg32(RP23XX_DMA_CTRL_TRIG(g_dvi.pixel_ch));
  if ((ctrl & RP23XX_DMA_CTRL_TRIG_AHB_ERROR) != 0)
    {
      putreg32(0, RP23XX_DMA_CTRL_TRIG(g_dvi.pixel_ch));
      putreg32(0, RP23XX_DMA_AL3_CTRL(g_dvi.command_ch));
      setbits_reg32(errors, RP23XX_DMA_CTRL_TRIG(g_dvi.pixel_ch));
      g_dvi.power = 0;
      return OK;
    }

  putreg32((uintptr_t)g_dma_commands,
           RP23XX_DMA_AL3_READ_ADDR_TRIG(g_dvi.command_ch));

  return OK;
}

static void rp23xx_dvi_configure_access(void)
{
  setbits_reg32(RP23XX_ACCESSCTRL_HSTX_DBG_MASK |
                RP23XX_ACCESSCTRL_HSTX_DMA_MASK |
                RP23XX_ACCESSCTRL_HSTX_CORE0_MASK |
                RP23XX_ACCESSCTRL_HSTX_SP_MASK |
                RP23XX_ACCESSCTRL_HSTX_SU_MASK |
                RP23XX_ACCESSCTRL_HSTX_NSP_MASK |
                RP23XX_ACCESSCTRL_HSTX_NSU_MASK,
                RP23XX_ACCESSCTRL_HSTX);
}

static void rp23xx_dvi_configure_hstx(void)
{
  const unsigned int pins[] =
    {
      BOARD_DVI_D0_PIN, BOARD_DVI_D1_PIN,
      BOARD_DVI_D2_PIN, BOARD_DVI_D3_PIN,
      BOARD_DVI_D4_PIN, BOARD_DVI_D5_PIN,
      BOARD_DVI_D6_PIN, BOARD_DVI_D7_PIN
    };
  const unsigned int lane_pin[] =
    {
      BOARD_DVI_D3_PIN, BOARD_DVI_D5_PIN, BOARD_DVI_D7_PIN
    };
  uint32_t clkdiv = BOARD_HSTX_FREQ / RP23XX_DVI_PIXEL_CLOCK;
  unsigned int i;

  if (clkdiv < 1)
    {
      clkdiv = 1;
    }

  putreg32((4u << HSTX_EXPAND_TMDS_L2_NBITS_SHIFT) |
           (8u << HSTX_EXPAND_TMDS_L2_ROT_SHIFT) |
           (5u << HSTX_EXPAND_TMDS_L1_NBITS_SHIFT) |
           (3u << HSTX_EXPAND_TMDS_L1_ROT_SHIFT) |
           (4u << HSTX_EXPAND_TMDS_L0_NBITS_SHIFT) |
           29u,
           RP23XX_HSTX_CTRL_EXPAND_TMDS);

  putreg32((RP23XX_DVI_OUTPUT_SCALING <<
            HSTX_EXPAND_ENC_N_SHIFTS_SHIFT) |
           (16u << HSTX_EXPAND_ENC_SHIFT_SHIFT) |
           (1u << HSTX_EXPAND_RAW_N_SHIFTS_SHIFT),
           RP23XX_HSTX_CTRL_EXPAND_SHIFT);

  putreg32(0, RP23XX_HSTX_CTRL_CSR);
  putreg32(RP23XX_HSTX_CTRL_CSR_EXPAND_EN |
           (clkdiv << HSTX_CSR_CLKDIV_SHIFT) |
           (5u << HSTX_CSR_N_SHIFTS_SHIFT) |
           (2u << HSTX_CSR_SHIFT_SHIFT) |
           RP23XX_HSTX_CTRL_CSR_EN,
           RP23XX_HSTX_CTRL_CSR);

  i = BOARD_DVI_D1_PIN - BOARD_DVI_D0_PIN;
  putreg32(RP23XX_HSTX_CTRL_BIT_CLK_MASK, RP23XX_HSTX_CTRL_BIT(i));
  putreg32(RP23XX_HSTX_CTRL_BIT_CLK_MASK | RP23XX_HSTX_CTRL_BIT_INV_MASK,
           RP23XX_HSTX_CTRL_BIT(i ^ 1));

  for (i = 0; i < 3; i++)
    {
      const unsigned int bit = lane_pin[i] - BOARD_DVI_D0_PIN;
      const uint32_t sel = ((i * 10u) |
                            ((i * 10u + 1u) << HSTX_BIT_SEL_N_SHIFT));

      putreg32(sel, RP23XX_HSTX_CTRL_BIT(bit));
      putreg32(sel | RP23XX_HSTX_CTRL_BIT_INV_MASK,
               RP23XX_HSTX_CTRL_BIT(bit ^ 1));
    }

  for (i = 0; i < nitems(pins); i++)
    {
      rp23xx_gpio_set_function(pins[i], RP23XX_GPIO_FUNC_HSTX);
      rp23xx_gpio_set_drive_strength(pins[i],
                                     RP23XX_PADS_BANK0_GPIO_DRIVE_4MA);
    }
}

static int rp23xx_dvi_configure_dma(void)
{
  uint32_t bitmask;
  uint32_t command_ctrl;
  int ret;

  g_dvi.pixel_dma = rp23xx_dmachannel();
  g_dvi.command_dma = rp23xx_dmachannel();

  if (g_dvi.pixel_dma == NULL || g_dvi.command_dma == NULL)
    {
      if (g_dvi.pixel_dma != NULL)
        {
          rp23xx_dmafree(g_dvi.pixel_dma);
          g_dvi.pixel_dma = NULL;
        }

      if (g_dvi.command_dma != NULL)
        {
          rp23xx_dmafree(g_dvi.command_dma);
          g_dvi.command_dma = NULL;
        }

      return -ENOMEM;
    }

  g_dvi.pixel_ch = rp23xx_dvi_dma_channel(g_dvi.pixel_dma);
  g_dvi.command_ch = rp23xx_dvi_dma_channel(g_dvi.command_dma);
  bitmask = (1u << g_dvi.pixel_ch) | (1u << g_dvi.command_ch);

  clrbits_reg32(bitmask, RP23XX_DMA_INTE0);
  putreg32(bitmask, RP23XX_DMA_INTS0);
  putreg32(bitmask, RP23XX_DMA_INTS2);

  rp23xx_dvi_build_command_list();

  command_ctrl = RP23XX_DMA_CTRL_TRIG_READ_ERROR |
                 RP23XX_DMA_CTRL_TRIG_WRITE_ERROR |
                 RP23XX_DMA_CTRL_TRIG_INCR_READ |
                 RP23XX_DMA_CTRL_TRIG_INCR_WRITE |
                 RP23XX_DMA_CTRL_TRIG_HIGH_PRIORITY |
                 RP23XX_DMA_CTRL_TRIG_RING_SEL |
                 (4u << RP23XX_DMA_CTRL_TRIG_RING_SIZE_SHIFT) |
                 (RP23XX_DMA_SIZE_WORD <<
                  RP23XX_DMA_CTRL_TRIG_DATA_SIZE_SHIFT) |
                 (g_dvi.command_ch <<
                  RP23XX_DMA_CTRL_TRIG_CHAIN_TO_SHIFT) |
                 RP23XX_DMA_CTRL_TRIG_TREQ_SEL_PERMANENT |
                 RP23XX_DMA_CTRL_TRIG_EN;

  putreg32(command_ctrl, RP23XX_DMA_AL3_CTRL(g_dvi.command_ch));
  putreg32(RP23XX_DMA_AL3_CTRL(g_dvi.pixel_ch),
           RP23XX_DMA_AL3_WRITE_ADDR(g_dvi.command_ch));
  putreg32(4, RP23XX_DMA_AL3_TRANS_COUNT(g_dvi.command_ch));

  putreg32(1u << g_dvi.pixel_ch, RP23XX_DMA_INTE2);
  ret = irq_attach(RP23XX_DMA_IRQ_2, rp23xx_dvi_dma_interrupt, NULL);
  if (ret < 0)
    {
      rp23xx_dmafree(g_dvi.pixel_dma);
      rp23xx_dmafree(g_dvi.command_dma);
      g_dvi.pixel_dma = NULL;
      g_dvi.command_dma = NULL;
      return ret;
    }

  up_enable_irq(RP23XX_DMA_IRQ_2);

  setbits_reg32(RP23XX_DVI_BUS_PRIORITY_DMA_W |
                RP23XX_DVI_BUS_PRIORITY_DMA_R,
                RP23XX_DVI_BUS_PRIORITY);

  rp23xx_dvi_dma_interrupt(RP23XX_DMA_IRQ_2, NULL, NULL);
  return OK;
}

static int rp23xx_dvi_start(void)
{
  int ret;

  if (g_dvi.streaming)
    {
      g_dvi.power = 1;
      return OK;
    }

  rp23xx_dvi_configure_access();
  rp23xx_dvi_configure_hstx();

  ret = rp23xx_dvi_configure_dma();
  if (ret < 0)
    {
      putreg32(0, RP23XX_HSTX_CTRL_CSR);
      g_dvi.power = 0;
      return ret;
    }

  g_dvi.streaming = true;
  g_dvi.power = 1;
  return OK;
}

static int rp23xx_dvi_getvideoinfo(FAR struct fb_vtable_s *vtable,
                                   FAR struct fb_videoinfo_s *vinfo)
{
  if (vinfo == NULL)
    {
      return -EINVAL;
    }

  memset(vinfo, 0, sizeof(*vinfo));
  vinfo->fmt = FB_FMT_RGB16_565;
  vinfo->xres = RP23XX_DVI_FB_WIDTH;
  vinfo->yres = RP23XX_DVI_FB_HEIGHT;
  vinfo->nplanes = 1;

  return OK;
}

static int rp23xx_dvi_getplaneinfo(FAR struct fb_vtable_s *vtable,
                                   int planeno,
                                   FAR struct fb_planeinfo_s *pinfo)
{
  if (pinfo == NULL)
    {
      return -EINVAL;
    }

  if (planeno != 0)
    {
      return -EINVAL;
    }

  memset(pinfo, 0, sizeof(*pinfo));
  pinfo->fbmem = g_framebuffer;
  pinfo->fblen = RP23XX_DVI_FRAME_BYTES;
  pinfo->stride = RP23XX_DVI_FB_WIDTH * 2;
  pinfo->display = 0;
  pinfo->bpp = RP23XX_DVI_BPP;
  pinfo->xres_virtual = RP23XX_DVI_FB_WIDTH;
  pinfo->yres_virtual = RP23XX_DVI_FB_HEIGHT;

  return OK;
}

static int rp23xx_dvi_open(FAR struct fb_vtable_s *vtable)
{
  return rp23xx_dvi_start();
}

static int rp23xx_dvi_close(FAR struct fb_vtable_s *vtable)
{
  return OK;
}

#ifdef CONFIG_FB_UPDATE
static int rp23xx_dvi_updatearea(FAR struct fb_vtable_s *vtable,
                                 FAR const struct fb_area_s *area)
{
  return OK;
}
#endif

static int rp23xx_dvi_setframerate(FAR struct fb_vtable_s *vtable,
                                   int rate)
{
  return rate == 60 ? OK : -EINVAL;
}

static int rp23xx_dvi_getframerate(FAR struct fb_vtable_s *vtable)
{
  return 60;
}

static int rp23xx_dvi_getpower(FAR struct fb_vtable_s *vtable)
{
  return g_dvi.power;
}

static int rp23xx_dvi_setpower(FAR struct fb_vtable_s *vtable, int power)
{
  if (power > 0)
    {
      return rp23xx_dvi_start();
    }

  g_dvi.power = 0;
  return OK;
}

static int rp23xx_dvi_ioctl(FAR struct fb_vtable_s *vtable, int cmd,
                            unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int up_fbinitialize(int display)
{
  if (display != 0)
    {
      return -EINVAL;
    }

  if (g_dvi.initialized)
    {
      return OK;
    }

  memset(g_framebuffer, 0, sizeof(g_framebuffer));

  g_dvi.initialized = true;
  g_dvi.power = 0;
  return OK;
}

FAR struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  if (display != 0 || vplane != 0)
    {
      return NULL;
    }

  return &g_dvi.vtable;
}

void up_fbuninitialize(int display)
{
}

#endif /* CONFIG_RP23XX_HSTX_DVI */
