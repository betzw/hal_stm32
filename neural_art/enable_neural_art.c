#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include <soc.h>

#include "stm32n6xx_hal.h"
#include "mcu_cache.h"
#include "npu_cache.h"

LOG_MODULE_REGISTER(enable_neural_art, CONFIG_NEURAL_ART_LOG_LEVEL);

/*******************************************************************************
 * Private
 ******************************************************************************/

#define _FLASH_ENABLE 1
#define _PSRAM_ENABLE 1

static void RISAF_Config(int flash_en, int psram_en)
{
  /* Disable region before programming */
  RISAF2_S->REG[0].CFGR &= ~RISAF_REGx_CFGR_BREN;
  RISAF2_S->REG[1].CFGR &= ~RISAF_REGx_CFGR_BREN;
  RISAF3_S->REG[0].CFGR &= ~RISAF_REGx_CFGR_BREN;
  RISAF3_S->REG[1].CFGR &= ~RISAF_REGx_CFGR_BREN;
  RISAF4_S->REG[0].CFGR &= ~RISAF_REGx_CFGR_BREN;
  RISAF4_S->REG[1].CFGR &= ~RISAF_REGx_CFGR_BREN;
  RISAF5_S->REG[0].CFGR &= ~RISAF_REGx_CFGR_BREN;
  RISAF5_S->REG[1].CFGR &= ~RISAF_REGx_CFGR_BREN;
  RISAF6_S->REG[0].CFGR &= ~RISAF_REGx_CFGR_BREN;
  RISAF6_S->REG[1].CFGR &= ~RISAF_REGx_CFGR_BREN;
  RISAF7_S->REG[0].CFGR &= ~RISAF_REGx_CFGR_BREN;
  RISAF7_S->REG[1].CFGR &= ~RISAF_REGx_CFGR_BREN;

  /* NPU access to (AXI-SRAM1) */
  RISAF2_S->REG[0].CIDCFGR = 0x000F000F; /* RW for everyone */
  RISAF2_S->REG[0].ENDR = 0xFFFFFFFF;    /* all-encompassing */
  RISAF2_S->REG[0].CFGR = 0x00000101;    /* enabled, secure, unprivileged for everyone */
  RISAF2_S->REG[1].CIDCFGR = 0x00FF00FF; /* RW for everyone */
  RISAF2_S->REG[1].ENDR = 0xFFFFFFFF;    /* all-encompassing */
  RISAF2_S->REG[1].CFGR = 0x00000001;    /* enabled, non-secure, unprivileged for everyone */

  /* Allow memory access for DCMIPP to Buffer0 (AXI-SRAM2) */
  RISAF3_S->REG[0].CIDCFGR = 0x000F000F; /* RW for everyone */
  RISAF3_S->REG[0].ENDR = 0xFFFFFFFF;    /* all-encompassing */
  RISAF3_S->REG[0].CFGR = 0x00000101;    /* enabled, secure, unprivileged for everyone */
  RISAF3_S->REG[1].CIDCFGR = 0x00FF00FF; /* RW for everyone */
  RISAF3_S->REG[1].ENDR = 0xFFFFFFFF;    /* all-encompassing */
  RISAF3_S->REG[1].CFGR = 0x00000001;    /* enabled, non-secure, unprivileged for everyone */

  // Allow memory access to NPU master 0
  RISAF4_S->REG[0].CIDCFGR = 0x000F000F; /* RW for everyone */
  RISAF4_S->REG[0].ENDR = 0xFFFFFFFF;    /* all-encompassing */
  RISAF4_S->REG[0].CFGR = 0x00000101;    /* enabled, secure, unprivileged for everyone */
  RISAF4_S->REG[1].CIDCFGR = 0x00FF00FF; /* RW for everyone */
  RISAF4_S->REG[1].ENDR = 0xFFFFFFFF;    /* all-encompassing */
  RISAF4_S->REG[1].CFGR = 0x00000001;    /* enabled, non-secure, unprivileged for everyone */

  // Allow memory access to NPU master 1
  RISAF5_S->REG[0].CIDCFGR = 0x000F000F; /* RW for everyone */
  RISAF5_S->REG[0].ENDR = 0xFFFFFFFF;    /* all-encompassing */
  RISAF5_S->REG[0].CFGR = 0x00000101;    /* enabled, secure, unprivileged for everyone */
  RISAF5_S->REG[1].CIDCFGR = 0x00FF00FF; /* RW for everyone */
  RISAF5_S->REG[1].ENDR = 0xFFFFFFFF;    /* all-encompassing */
  RISAF5_S->REG[1].CFGR = 0x00000001;    /* enabled, non-secure, unprivileged for everyone */

  /* Allow memory access for DCMIPP to Buffer1 (AXI-SRAM3 NPU) */
  RISAF6_S->REG[0].CIDCFGR = 0x000F000F; /* RW for everyone */
  RISAF6_S->REG[0].ENDR = 0xFFFFFFFF;    /* all-encompassing */
  RISAF6_S->REG[0].CFGR = 0x00000101;    /* enabled, secure, unprivileged for everyone */
  RISAF6_S->REG[1].CIDCFGR = 0x00FF00FF; /* RW for everyone */
  RISAF6_S->REG[1].ENDR = 0xFFFFFFFF;    /* all-encompassing */
  RISAF6_S->REG[1].CFGR = 0x00000001;    /* enabled, non-secure, unprivileged for everyone */

  /* Allow memory access to AXI-SRAM1 FLEXMEM */
  RISAF7_S->REG[0].CIDCFGR = 0x000F000F; /* RW for everyone */
  RISAF7_S->REG[0].ENDR = 0xFFFFFFFF;    /* all-encompassing */
  RISAF7_S->REG[0].CFGR = 0x00000101;    /* enabled, secure, unprivileged for everyone */
  RISAF7_S->REG[1].CIDCFGR = 0x00FF00FF; /* RW for everyone */
  RISAF7_S->REG[1].ENDR = 0xFFFFFFFF;    /* all-encompassing */
  RISAF7_S->REG[1].CFGR = 0x00000001;    /* enabled, non-secure, unprivileged for everyone */

  /* Do not enable PSRAM if device is not present to avoid application crash */
  if (psram_en)
  {
    /* Allow memory access to memory mapped XSPI1 region (PSRAM) */
    RISAF11_S->REG[0].CIDCFGR = 0x000F000F; /* RW for everyone */
    RISAF11_S->REG[0].ENDR = 0xFFFFFFFF;    /* all-encompassing */
    RISAF11_S->REG[0].CFGR = 0x00000101;    /* enabled, secure, unprivileged for everyone */
    RISAF11_S->REG[1].CIDCFGR = 0x00FF00FF; /* RW for everyone */
    RISAF11_S->REG[1].ENDR = 0xFFFFFFFF;    /* all-encompassing */
    RISAF11_S->REG[1].CFGR = 0x00000001;    /* enabled, non-secure, unprivileged for everyone */
  }

  if (flash_en)
  {
    /* Allow access to memory mapped XSPI2 region (NOR Flash) */
    RISAF12_S->REG[0].CIDCFGR = 0x000F000F; /* RW for everyone */
    RISAF12_S->REG[0].ENDR = 0xFFFFFFFF;    /* all-encompassing */
    RISAF12_S->REG[0].CFGR = 0x00000101;    /* enabled, secure, unprivileged for everyone */
    RISAF12_S->REG[1].CIDCFGR = 0x00FF00FF; /* RW for everyone */
    RISAF12_S->REG[1].ENDR = 0xFFFFFFFF;    /* all-encompassing */
    RISAF12_S->REG[1].CFGR = 0x00000001;    /* enabled, non-secure, unprivileged for everyone */
  }
}
 
/*******************************************************************************
 * Initialization
 ******************************************************************************/

static int enable_neural_art_init(void)
{
  /* Enable Cache AXI clocks */
  __HAL_RCC_CACHEAXI_CLK_ENABLE();
  RCC->MEMENR |= RCC_MEMENR_CACHEAXIRAMEN;

  /* Enable NPU clock */
  __HAL_RCC_NPU_CLK_ENABLE();
  __HAL_RCC_NPU_FORCE_RESET();
  __HAL_RCC_NPU_RELEASE_RESET();

  /* Ensure SAU is disabled to reach optimal performance for CPU fetch in NS region (not strictly necessary) */
  TZ_SAU_Disable();

  /* Enable Secure access for NPU (see Mark Trimmer) */
  RISAF_Config(_FLASH_ENABLE, _PSRAM_ENABLE);

  /* Leave clocks enabled in Low Power mode or ATON will be gated during wfe */
  RCC->BUSLPENR = 0xFFFFFFFF;
  RCC->MISCLPENR = 0xFFFFFFFF;
  RCC->MEMLPENR = 0xFFFFFFFF;
  RCC->AHB1LPENR = 0xFFFFFFFF;
  RCC->AHB2LPENR = 0xFFFFFFFF;
  RCC->AHB3LPENR = 0xFFFFFFFF;
  RCC->AHB4LPENR = 0xFFFFFFFF;
  RCC->AHB5LPENR = 0xFFFFFFFF;
  RCC->APB1LPENR1 = 0xFFFFFFFF;
  RCC->APB1LPENR2 = 0xFFFFFFFF;
  RCC->APB2LPENR = 0xFFFFFFFF;
  RCC->APB3LPENR = 0xFFFFFFFF;
  RCC->APB4LPENR1 = 0xFFFFFFFF;
  RCC->APB4LPENR2 = 0xFFFFFFFF;
  RCC->APB5LPENR = 0xFFFFFFFF;

  /* Data Synchronization Barrier */
  __DSB();

  LOG_INF("NPU enabled successfully");
  
  /* Enable caches (as this is expected to be the situation when `main()` is called) */
  SCB_EnableICache();
  mcu_cache_enable();
  npu_cache_enable();

  return 0;
}

SYS_INIT(enable_neural_art_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
