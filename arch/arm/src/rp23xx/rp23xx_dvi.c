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
#include <nuttx/kmalloc.h>
#include <nuttx/video/fb.h>

#include <arch/board/board.h>
#include <arch/rp23xx/irq.h>

#include "arm_internal.h"

#include "rp23xx_dmac.h"
#include "rp23xx_gpio.h"

#include "hardware/rp23xx_dma.h"
#include "hardware/rp23xx_dreq.h"
#include "hardware/rp23xx_hstx_ctrl.h"
#include "hardware/rp23xx_hstx_fifo.h"
#include "hardware/rp23xx_memorymap.h"
#include "hardware/rp23xx_pads_bank0.h"
#include "hardware/rp23xx_resets.h"
#include "hardware/rp23xx_watchdog.h"

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

#define RP23XX_DVI_FB_WIDTH              320
#define RP23XX_DVI_FB_HEIGHT             240
#define RP23XX_DVI_OUTPUT_SCALING        CONFIG_RP23XX_HSTX_DVI_OUTPUT_SCALING
#define RP23XX_DVI_BPP                   16
#define RP23XX_DVI_SCANOUT_FRAMEBUFFER   0
#define RP23XX_DVI_SCANOUT_BLACK         1

#if RP23XX_DVI_OUTPUT_SCALING < 1 || RP23XX_DVI_OUTPUT_SCALING > 2
#  error "RP23XX HSTX DVI output scaling must be 1 or 2"
#endif

#define H_BORDER                         ((H_ACTIVE_PIXELS - \
                                          RP23XX_DVI_FB_WIDTH * \
                                          RP23XX_DVI_OUTPUT_SCALING) / 2)
#define V_BORDER                         ((V_ACTIVE_LINES - \
                                          RP23XX_DVI_FB_HEIGHT * \
                                          RP23XX_DVI_OUTPUT_SCALING) / 2)
#define RP23XX_DVI_CONTENT_LINES         (RP23XX_DVI_FB_HEIGHT * \
                                          RP23XX_DVI_OUTPUT_SCALING)

#define VSYNC_LEN                        6
#if H_BORDER > 0
#  define VACTIVE_PRE_LEN                11
#else
#  define VACTIVE_PRE_LEN                9
#endif
#define VACTIVE_POST_LEN                 2
#define VACTIVE_BLACK_LEN                10
#define DMA_CMD_BLOCK_WORDS              4
#ifdef CONFIG_RP23XX_HSTX_DVI_BLACKOUT
#  define DMA_CMD_BUF_WORDS              ((V_TOTAL_LINES * \
                                          DMA_CMD_BLOCK_WORDS) + \
                                          DMA_CMD_BLOCK_WORDS)
#else
#define DMA_CMD_CONTENT_WORDS            (2 * DMA_CMD_BLOCK_WORDS + \
                                          (H_BORDER > 0 ? \
                                           DMA_CMD_BLOCK_WORDS : 0))

/* Every scanline needs one control block. Content lines need two blocks when
 * the framebuffer fills the active width, or three when diagnostic centering
 * adds left/right borders. Add one terminating block to generate the
 * end-of-frame DMA interrupt.
 */

#define DMA_CMD_BUF_WORDS                (((V_TOTAL_LINES - \
                                          RP23XX_DVI_CONTENT_LINES) * \
                                          DMA_CMD_BLOCK_WORDS) + \
                                          (RP23XX_DVI_CONTENT_LINES * \
                                          DMA_CMD_CONTENT_WORDS) + \
                                          DMA_CMD_BLOCK_WORDS)
#endif

#define RP23XX_DVI_FRAME_BYTES           (RP23XX_DVI_FB_WIDTH * \
                                          RP23XX_DVI_FB_HEIGHT * 2)

#define RP23XX_DVI_PIXEL_CLOCK           25200000u
#define RP23XX_DVI_CLOCKDIV              ((BOARD_HSTX_FREQ + \
                                          (RP23XX_DVI_PIXEL_CLOCK / 2)) / \
                                          RP23XX_DVI_PIXEL_CLOCK)
#define RP23XX_DVI_ACTUAL_PIXEL_CLOCK    (BOARD_HSTX_FREQ / \
                                          RP23XX_DVI_CLOCKDIV)
#define RP23XX_DVI_TREQL_HSTX            RP23XX_DMA_DREQ_HSTX
#define RP23XX_DVI_COLORBAR_WIDTH        (RP23XX_DVI_FB_WIDTH / 8)
#define RP23XX_DVI_RESET_TIMEOUT         1000000u
#define RP23XX_DVIIOC_GETINFO            _FBIOC(0x00f0)

#define RP23XX_DVI_STAGE_IDLE            0x44563030u /* "DV00" */
#define RP23XX_DVI_STAGE_START           0x44565354u /* "DVST" */
#define RP23XX_DVI_STAGE_HSTX_ENTER      0x44564830u /* "DVH0" */
#define RP23XX_DVI_STAGE_HSTX_DONE       0x44564831u /* "DVH1" */
#define RP23XX_DVI_STAGE_DMA_ENTER       0x44564430u /* "DVD0" */
#define RP23XX_DVI_STAGE_DMA_ALLOC_PIXEL 0x44564150u /* "DVAP" */
#define RP23XX_DVI_STAGE_DMA_ALLOC_CMD   0x44564143u /* "DVAC" */
#define RP23XX_DVI_STAGE_DMA_ABORT       0x44564142u /* "DVAB" */
#define RP23XX_DVI_STAGE_DMA_COMMANDS    0x4456434cu /* "DVCL" */
#define RP23XX_DVI_STAGE_DMA_IRQ         0x44564952u /* "DVIR" */
#define RP23XX_DVI_STAGE_DMA_KICK        0x44564b49u /* "DVKI" */
#define RP23XX_DVI_STAGE_DMA_DONE        0x44564431u /* "DVD1" */
#define RP23XX_DVI_STAGE_STARTED         0x44564f4bu /* "DVOK" */

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
  volatile uint32_t frame_count;
  volatile uint32_t irq_count;
  volatile uint32_t error_count;
  int power;
  bool initialized;
  bool irq_attached;
  bool streaming;
};

struct rp23xx_dvi_info_s
{
  uint32_t framebuffer_width;
  uint32_t framebuffer_height;
  uint32_t output_width;
  uint32_t output_height;
  uint32_t output_scaling;
  uint32_t bpp;
  uint32_t frame_bytes;
  uint32_t sys_clock;
  uint32_t peri_clock;
  uint32_t hstx_clock;
  uint32_t pixel_clock;
  uint32_t target_pixel_clock;
  uint32_t clkdiv;
  uint32_t scanout_mode;
  uint32_t power;
  uint32_t streaming;
  uint32_t dma_allocated;
  uint32_t pixel_ch;
  uint32_t command_ch;
  uint32_t irq_attached;
  uint32_t frame_count;
  uint32_t irq_count;
  uint32_t error_count;
  uint32_t hstx_csr;
  uint32_t hstx_fifo_stat;
  uint32_t dma_intr;
  uint32_t dma_inte2;
  uint32_t dma_ints2;
  uint32_t pixel_ctrl;
  uint32_t pixel_read_addr;
  uint32_t pixel_write_addr;
  uint32_t pixel_trans_count;
  uint32_t command_ctrl;
  uint32_t command_read_addr;
  uint32_t command_write_addr;
  uint32_t command_trans_count;
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

#ifndef CONFIG_RP23XX_HSTX_DVI_BLACKOUT
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
#if H_BORDER > 0
  HSTX_CMD_TMDS_REPEAT | H_BORDER,
  0x00000000,
#endif
  HSTX_CMD_TMDS | (RP23XX_DVI_FB_WIDTH * RP23XX_DVI_OUTPUT_SCALING)
};

static uint32_t g_vactive_post[VACTIVE_POST_LEN] =
{
  HSTX_CMD_TMDS_REPEAT | H_BORDER,
  0x00000000
};
#endif

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
static FAR uint16_t *g_framebuffer;

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

static void rp23xx_dvi_stage(uint32_t stage)
{
  putreg32(stage, RP23XX_WATCHDOG_SCRATCH(2));
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
    RP23XX_DMA_CTRL_TRIG_EN;
#ifndef CONFIG_RP23XX_HSTX_DVI_BLACKOUT
  const uint32_t dma_pixel_ctrl =
    dma_ctrl_base |
    (RP23XX_DMA_SIZE_HALFWORD << RP23XX_DMA_CTRL_TRIG_DATA_SIZE_SHIFT);
#endif
  const uint32_t dma_ctrl =
    dma_ctrl_base |
    (RP23XX_DMA_SIZE_WORD << RP23XX_DMA_CTRL_TRIG_DATA_SIZE_SHIFT);
  const uint32_t dma_write_addr = RP23XX_HSTX_FIFO_FIFO;

  const unsigned int vsync_start = 0;
  const unsigned int vsync_end = V_SYNC_WIDTH;
  const unsigned int backporch_end = vsync_end + V_BACK_PORCH;
  const unsigned int active_start = backporch_end;
  const unsigned int frontporch_start = V_TOTAL_LINES - V_FRONT_PORCH;
#ifndef CONFIG_RP23XX_HSTX_DVI_BLACKOUT
  const unsigned int content_start = active_start + V_BORDER;
  const unsigned int content_end =
    content_start + RP23XX_DVI_CONTENT_LINES;
#endif
  uint32_t cw = 0;
  unsigned int line;

  DEBUGASSERT(g_framebuffer != NULL);

  for (line = 0; line < V_TOTAL_LINES; line++)
    {
      if (line >= vsync_start && line < vsync_end)
        {
          rp23xx_dvi_push_block(&cw, dma_ctrl, dma_write_addr, VSYNC_LEN,
                                (uintptr_t)g_vblank_vsync_on);
        }
      else if (line >= active_start && line < frontporch_start)
        {
#ifdef CONFIG_RP23XX_HSTX_DVI_BLACKOUT
          rp23xx_dvi_push_block(&cw, dma_ctrl, dma_write_addr,
                                VACTIVE_BLACK_LEN,
                                (uintptr_t)g_vactive_black);
#else
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

              if (H_BORDER > 0)
                {
                  rp23xx_dvi_push_block(&cw, dma_ctrl, dma_write_addr,
                                        VACTIVE_POST_LEN,
                                        (uintptr_t)g_vactive_post);
                }
            }
          else
            {
              rp23xx_dvi_push_block(&cw, dma_ctrl, dma_write_addr,
                                    VACTIVE_BLACK_LEN,
                                    (uintptr_t)g_vactive_black);
            }
#endif
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

static void rp23xx_dvi_fill_colorbars(void)
{
  static const uint16_t colors[] =
    {
      0xffff, 0xffe0, 0x07ff, 0x07e0,
      0xf81f, 0xf800, 0x001f, 0x0000
    };
  unsigned int x;
  unsigned int y;

  for (y = 0; y < RP23XX_DVI_FB_HEIGHT; y++)
    {
      for (x = 0; x < RP23XX_DVI_FB_WIDTH; x++)
        {
          unsigned int bar = x / RP23XX_DVI_COLORBAR_WIDTH;

          if (bar >= nitems(colors))
            {
              bar = nitems(colors) - 1;
            }

          g_framebuffer[y * RP23XX_DVI_FB_WIDTH + x] = colors[bar];
        }
    }
}

static int rp23xx_dvi_dma_interrupt(int irq, void *context, void *arg)
{
  const uint32_t bit = 1u << g_dvi.pixel_ch;
  const uint32_t errors = RP23XX_DMA_CTRL_TRIG_READ_ERROR |
                          RP23XX_DMA_CTRL_TRIG_WRITE_ERROR;
  uint32_t ctrl;

  g_dvi.irq_count++;
  putreg32(bit, RP23XX_DMA_INTR);
  putreg32(bit, RP23XX_DMA_INTS2);

  ctrl = getreg32(RP23XX_DMA_CTRL_TRIG(g_dvi.pixel_ch));
  if ((ctrl & RP23XX_DMA_CTRL_TRIG_AHB_ERROR) != 0)
    {
      g_dvi.error_count++;
      putreg32(0, RP23XX_DMA_CTRL_TRIG(g_dvi.pixel_ch));
      putreg32(0, RP23XX_DMA_AL3_CTRL(g_dvi.command_ch));
      clrbits_reg32(bit, RP23XX_DMA_INTE2);
      setbits_reg32(errors, RP23XX_DMA_CTRL_TRIG(g_dvi.pixel_ch));
      g_dvi.power = 0;
      g_dvi.streaming = false;
      return OK;
    }

  putreg32((uintptr_t)g_dma_commands,
           RP23XX_DMA_AL3_READ_ADDR_TRIG(g_dvi.command_ch));
  g_dvi.frame_count++;

  return OK;
}

static void rp23xx_dvi_configure_access(void)
{
}

static int rp23xx_dvi_configure_hstx(void)
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
  uint32_t clkdiv = RP23XX_DVI_CLOCKDIV;
  unsigned int i;
  unsigned int timeout;

  putreg32(0, RP23XX_HSTX_CTRL_CSR);
  setbits_reg32(RP23XX_RESETS_RESET_HSTX, RP23XX_RESETS_RESET);
  clrbits_reg32(RP23XX_RESETS_RESET_HSTX, RP23XX_RESETS_RESET);
  timeout = RP23XX_DVI_RESET_TIMEOUT;
  while ((getreg32(RP23XX_RESETS_RESET_DONE) &
          RP23XX_RESETS_RESET_HSTX) == 0)
    {
      if (timeout-- == 0)
        {
          return -ETIMEDOUT;
        }
    }

  if (clkdiv < 1)
    {
      clkdiv = 1;
    }
  else if (clkdiv > 0x0f)
    {
      clkdiv = 0x0f;
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

  return OK;
}

static int rp23xx_dvi_abort_dma(void)
{
  uint32_t bitmask;
  unsigned int timeout;

  if (g_dvi.pixel_dma == NULL || g_dvi.command_dma == NULL)
    {
      return OK;
    }

  rp23xx_dvi_stage(RP23XX_DVI_STAGE_DMA_ABORT);
  bitmask = (1u << g_dvi.pixel_ch) | (1u << g_dvi.command_ch);

  clrbits_reg32(bitmask, RP23XX_DMA_INTE0);
  clrbits_reg32(bitmask, RP23XX_DMA_INTE1);
  clrbits_reg32(bitmask, RP23XX_DMA_INTE2);
  clrbits_reg32(bitmask, RP23XX_DMA_INTE3);

  putreg32(bitmask, RP23XX_DMA_CHAN_ABORT);
  timeout = RP23XX_DVI_RESET_TIMEOUT;
  while ((getreg32(RP23XX_DMA_CHAN_ABORT) & bitmask) != 0)
    {
      if (timeout-- == 0)
        {
          return -ETIMEDOUT;
        }
    }

  putreg32(0, RP23XX_DMA_CTRL_TRIG(g_dvi.pixel_ch));
  putreg32(0, RP23XX_DMA_CTRL_TRIG(g_dvi.command_ch));
  putreg32(0, RP23XX_DMA_AL3_CTRL(g_dvi.command_ch));

  putreg32(bitmask, RP23XX_DMA_INTR);
  putreg32(bitmask, RP23XX_DMA_INTS0);
  putreg32(bitmask, RP23XX_DMA_INTS1);
  putreg32(bitmask, RP23XX_DMA_INTS2);
  putreg32(bitmask, RP23XX_DMA_INTS3);

  return OK;
}

static int rp23xx_dvi_configure_dma(void)
{
  uint32_t bitmask;
  uint32_t command_ctrl;
  int ret;

  rp23xx_dvi_stage(RP23XX_DVI_STAGE_DMA_ENTER);
  if (g_dvi.pixel_dma == NULL)
    {
      rp23xx_dvi_stage(RP23XX_DVI_STAGE_DMA_ALLOC_PIXEL);
      g_dvi.pixel_dma = rp23xx_dmachannel();
      if (g_dvi.pixel_dma == NULL)
        {
          return -ENOMEM;
        }

      g_dvi.pixel_ch = rp23xx_dvi_dma_channel(g_dvi.pixel_dma);
    }

  if (g_dvi.command_dma == NULL)
    {
      rp23xx_dvi_stage(RP23XX_DVI_STAGE_DMA_ALLOC_CMD);
      g_dvi.command_dma = rp23xx_dmachannel();
      if (g_dvi.command_dma == NULL)
        {
          rp23xx_dmafree(g_dvi.pixel_dma);
          g_dvi.pixel_dma = NULL;
          return -ENOMEM;
        }

      g_dvi.command_ch = rp23xx_dvi_dma_channel(g_dvi.command_dma);
    }

  bitmask = (1u << g_dvi.pixel_ch) | (1u << g_dvi.command_ch);
  clrbits_reg32(bitmask, RP23XX_DMA_INTE0);
  putreg32(bitmask, RP23XX_DMA_INTS0);

  ret = rp23xx_dvi_abort_dma();
  if (ret < 0)
    {
      return ret;
    }

  rp23xx_dvi_stage(RP23XX_DVI_STAGE_DMA_COMMANDS);
  rp23xx_dvi_build_command_list();

  command_ctrl = RP23XX_DMA_CTRL_TRIG_READ_ERROR |
                 RP23XX_DMA_CTRL_TRIG_WRITE_ERROR |
                 RP23XX_DMA_CTRL_TRIG_INCR_READ |
                 RP23XX_DMA_CTRL_TRIG_INCR_WRITE |
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

  rp23xx_dvi_stage(RP23XX_DVI_STAGE_DMA_IRQ);
  putreg32(1u << g_dvi.pixel_ch, RP23XX_DMA_INTE2);
  if (!g_dvi.irq_attached)
    {
      ret = irq_attach(RP23XX_DMA_IRQ_2, rp23xx_dvi_dma_interrupt, NULL);
      if (ret < 0)
        {
          return ret;
        }

      g_dvi.irq_attached = true;
    }

  g_dvi.frame_count = 0;
  g_dvi.irq_count = 0;
  g_dvi.error_count = 0;

  up_enable_irq(RP23XX_DMA_IRQ_2);

  rp23xx_dvi_stage(RP23XX_DVI_STAGE_DMA_KICK);
  rp23xx_dvi_dma_interrupt(RP23XX_DMA_IRQ_2, NULL, NULL);
  rp23xx_dvi_stage(RP23XX_DVI_STAGE_DMA_DONE);
  return OK;
}

static int rp23xx_dvi_stop(void)
{
  int ret;

  up_disable_irq(RP23XX_DMA_IRQ_2);
  ret = rp23xx_dvi_abort_dma();

  putreg32(0, RP23XX_HSTX_CTRL_CSR);
  g_dvi.streaming = false;
  g_dvi.power = 0;

  return ret;
}

static int rp23xx_dvi_start(void)
{
  int ret;

  if (g_framebuffer == NULL)
    {
      return -ENODEV;
    }

  rp23xx_dvi_stage(RP23XX_DVI_STAGE_START);
  if (g_dvi.streaming)
    {
      g_dvi.power = 1;
      return OK;
    }

  rp23xx_dvi_configure_access();
  rp23xx_dvi_stage(RP23XX_DVI_STAGE_HSTX_ENTER);
  ret = rp23xx_dvi_configure_hstx();
  if (ret < 0)
    {
      putreg32(0, RP23XX_HSTX_CTRL_CSR);
      g_dvi.power = 0;
      return ret;
    }

  rp23xx_dvi_stage(RP23XX_DVI_STAGE_HSTX_DONE);
  ret = rp23xx_dvi_configure_dma();
  if (ret < 0)
    {
      putreg32(0, RP23XX_HSTX_CTRL_CSR);
      g_dvi.power = 0;
      return ret;
    }

  g_dvi.streaming = true;
  g_dvi.power = 1;
  rp23xx_dvi_stage(RP23XX_DVI_STAGE_STARTED);
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
  if (g_framebuffer == NULL)
    {
      return -ENODEV;
    }

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
  return OK;
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

  return rp23xx_dvi_stop();
}

static int rp23xx_dvi_ioctl(FAR struct fb_vtable_s *vtable, int cmd,
                            unsigned long arg)
{
  FAR struct rp23xx_dvi_info_s *info;

  if (cmd == RP23XX_DVIIOC_GETINFO)
    {
      info = (FAR struct rp23xx_dvi_info_s *)((uintptr_t)arg);
      if (info == NULL)
        {
          return -EINVAL;
        }

      memset(info, 0, sizeof(*info));
      info->framebuffer_width = RP23XX_DVI_FB_WIDTH;
      info->framebuffer_height = RP23XX_DVI_FB_HEIGHT;
      info->output_width = H_ACTIVE_PIXELS;
      info->output_height = V_ACTIVE_LINES;
      info->output_scaling = RP23XX_DVI_OUTPUT_SCALING;
      info->bpp = RP23XX_DVI_BPP;
      info->frame_bytes = RP23XX_DVI_FRAME_BYTES;
      info->sys_clock = BOARD_SYS_FREQ;
      info->peri_clock = BOARD_PERI_FREQ;
      info->hstx_clock = BOARD_HSTX_FREQ;
      info->pixel_clock = RP23XX_DVI_ACTUAL_PIXEL_CLOCK;
      info->target_pixel_clock = RP23XX_DVI_PIXEL_CLOCK;
      info->clkdiv = RP23XX_DVI_CLOCKDIV;
#ifdef CONFIG_RP23XX_HSTX_DVI_BLACKOUT
      info->scanout_mode = RP23XX_DVI_SCANOUT_BLACK;
#else
      info->scanout_mode = RP23XX_DVI_SCANOUT_FRAMEBUFFER;
#endif
      info->power = g_dvi.power;
      info->streaming = g_dvi.streaming ? 1 : 0;
      info->dma_allocated =
        g_dvi.pixel_dma != NULL && g_dvi.command_dma != NULL ? 1 : 0;
      info->pixel_ch = g_dvi.pixel_ch;
      info->command_ch = g_dvi.command_ch;
      info->irq_attached = g_dvi.irq_attached ? 1 : 0;
      info->frame_count = g_dvi.frame_count;
      info->irq_count = g_dvi.irq_count;
      info->error_count = g_dvi.error_count;
      info->hstx_csr = getreg32(RP23XX_HSTX_CTRL_CSR);
      info->hstx_fifo_stat = getreg32(RP23XX_HSTX_FIFO_STAT);
      info->dma_intr = getreg32(RP23XX_DMA_INTR);
      info->dma_inte2 = getreg32(RP23XX_DMA_INTE2);
      info->dma_ints2 = getreg32(RP23XX_DMA_INTS2);

      if (g_dvi.pixel_dma != NULL && g_dvi.command_dma != NULL)
        {
          info->pixel_ctrl = getreg32(RP23XX_DMA_CTRL_TRIG(g_dvi.pixel_ch));
          info->pixel_read_addr =
            getreg32(RP23XX_DMA_READ_ADDR(g_dvi.pixel_ch));
          info->pixel_write_addr =
            getreg32(RP23XX_DMA_WRITE_ADDR(g_dvi.pixel_ch));
          info->pixel_trans_count =
            getreg32(RP23XX_DMA_TRANS_COUNT(g_dvi.pixel_ch));
          info->command_ctrl =
            getreg32(RP23XX_DMA_CTRL_TRIG(g_dvi.command_ch));
          info->command_read_addr =
            getreg32(RP23XX_DMA_READ_ADDR(g_dvi.command_ch));
          info->command_write_addr =
            getreg32(RP23XX_DMA_WRITE_ADDR(g_dvi.command_ch));
          info->command_trans_count =
            getreg32(RP23XX_DMA_TRANS_COUNT(g_dvi.command_ch));
        }

      return OK;
    }

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

  g_framebuffer = kmm_memalign(4, RP23XX_DVI_FRAME_BYTES);
  if (g_framebuffer == NULL)
    {
      return -ENOMEM;
    }

  memset(g_framebuffer, 0, RP23XX_DVI_FRAME_BYTES);
  rp23xx_dvi_fill_colorbars();

  g_dvi.initialized = true;
  g_dvi.power = 0;
  rp23xx_dvi_stage(RP23XX_DVI_STAGE_IDLE);
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
