/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_heaps.c
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

#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/mm.h>

#include <stdint.h>

#include "arm_internal.h"
#include "rp23xx_psram.h"

#if defined(CONFIG_RP23XX_PSRAM)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static void * const psram_start = (void *)RP23XX_PSRAM_NOCACHE_BASE;

#if defined (CONFIG_RP23XX_PSRAM_HEAP_USER)
static bool g_psram_user_heap;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined (CONFIG_RP23XX_PSRAM_HEAP_USER)
static uintptr_t rp23xx_internal_heap_end(void)
{
#ifdef CONFIG_ARCH_PGPOOL_PBASE
  return CONFIG_ARCH_PGPOOL_PBASE;
#else
  return CONFIG_RAM_END;
#endif
}

static uintptr_t rp23xx_heap_align_up(uintptr_t value)
{
  return (value + 7) & ~(uintptr_t)7;
}

static uintptr_t rp23xx_heap_align_down(uintptr_t value)
{
  return value & ~(uintptr_t)7;
}

static size_t rp23xx_fallback_kheap_size(void)
{
  uintptr_t start = rp23xx_heap_align_up(g_idle_topstack);
  uintptr_t end = rp23xx_heap_align_down(rp23xx_internal_heap_end());
  size_t available = end - start;
  size_t requested = CONFIG_RP23XX_PSRAM_FALLBACK_KERNEL_HEAPSIZE;

  if (requested >= available)
    {
      requested = available / 2;
    }

  return requested & ~(size_t)7;
}

static void rp23xx_allocate_internal_fallback_heap(void **heap_start,
                                                   size_t *heap_size)
{
  uintptr_t start = rp23xx_heap_align_up(g_idle_topstack);
  uintptr_t end = rp23xx_heap_align_down(rp23xx_internal_heap_end());
  size_t kheap_size = rp23xx_fallback_kheap_size();
  uintptr_t user_start = start + kheap_size;

  *heap_start = (void *)user_start;
  *heap_size = end - user_start;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#if defined(CONFIG_RP23XX_PSRAM_HEAP_SEPARATE)
static struct mm_heap_s *g_psramheap;
#endif

#if defined(CONFIG_RP23XX_PSRAM_HEAP_SINGLE)

#if defined(CONFIG_MM_KERNEL_HEAP)
#error cannot use CONFIG_MM_KERNEL_HEAP with single heap
#endif

#if CONFIG_MM_REGIONS > 1
void arm_addregion(void)
{
  size_t size = rp23xx_psramconfig();

  /* Add the PSRAM region to main heap */

  kumm_addregion(psram_start, size);
}
#endif

#elif defined (CONFIG_RP23XX_PSRAM_HEAP_USER)

#if !defined(CONFIG_MM_KERNEL_HEAP)
#error MM_KERNEL_HEAP is required for separate kernel heap
#endif

/* Use the internal SRAM as the kernel heap */

void up_allocate_kheap(void **heap_start, size_t *heap_size)
{
  *heap_start = (void *)rp23xx_heap_align_up(g_idle_topstack);

  if (!g_psram_user_heap)
    {
      *heap_size = rp23xx_fallback_kheap_size();
    }
  else
    {
      *heap_size = rp23xx_internal_heap_end() - (uintptr_t)*heap_start;
    }
}

/* Use the external PSRAM as the default user heap */

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  size_t size = rp23xx_psramconfig();

  if (size > 0)
    {
      g_psram_user_heap = true;
      *heap_start = psram_start;
      *heap_size = size;
    }
  else
    {
      g_psram_user_heap = false;
      rp23xx_allocate_internal_fallback_heap(heap_start, heap_size);
    }
}

#elif defined (CONFIG_RP23XX_PSRAM_HEAP_SEPARATE)

#if !defined(CONFIG_ARCH_HAVE_EXTRA_HEAPS)
#error ARCH_HAVE_EXTRA_HEAPS is required for multiple heaps
#endif

void up_extraheaps_init(void)
{
  size_t size = rp23xx_psramconfig();

  g_psramheap = mm_initialize("psram", psram_start, size);
}
#endif

#endif
