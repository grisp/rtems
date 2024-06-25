/* SPDX-License-Identifier: BSD-2-Clause */

/**
 * @file
 *
 * @ingroup RTEMSBSPsARMSTM32U5
 *
 * @brief This file provides the implementation of OctoSPI initalization of @ref
 * RTEMSBSPsARMSTM32U5.
 */

/*
 * Copyright (C) 2024 embedded brains GmbH & Co. KG
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <bsp.h>
#include <bsp/start.h>
#include <stm32u5/hal.h>

/* IS66 config register */
#define IS66_CFG_DEEP_PWRDWN_DIS       (1u << 15)
#define IS66_CFG_ODS(x)                (((x) & 0x07) << 12)
#define IS66_CFG_DQSM_READ_PRECYCLE_EN (1u << 8)
#define IS66_CFG_LATENCY_CTR(x)        (((x) & 0x0F) << 4)
#define IS66_CFG_ACCESS_LATENCY_FIXED  (1u << 3)
#define IS66_CFG_BURST_LENGTH(x)       (((x) & 0x03) << 0)

#define IS66_CFG_RESERVED_MASK         ((7u << 9) | (1 << 2))

/* Settings */
#define LATENCY_2CL 8 /* double latency that is used during refreshes */
#define DUMMY_CYCLES (LATENCY_2CL - 1)

#define RAM_CONFIG_REGISTER ( \
                            IS66_CFG_DEEP_PWRDWN_DIS | \
                            IS66_CFG_ODS(7) | \
                            IS66_CFG_LATENCY_CTR((LATENCY_2CL/2)-3) | \
                            IS66_CFG_ACCESS_LATENCY_FIXED |\
                            IS66_CFG_BURST_LENGTH(2) | \
                            0 )

static uint16_t BSP_START_TEXT_SECTION ram_config_register_read(OSPI_HandleTypeDef *hospi)
{
  OSPI_RegularCmdTypeDef   sCommand = {0};
  uint16_t reg;

  /*
   * Note:
   * - If you read more bytes, the RAM will just continue to push out the
   *   register content again and again.
   * - If DQS is enabled, the first data will be read, regardless whether all
   *   dummy cycles are already sent or not.
   * - In default settings, the RAM adds a dummy DQS before that first data. So
   *   the first value is not valid. Therefore we have to read two times.
   */

  /* Initialize the read command */
  sCommand.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId = HAL_OSPI_FLASH_ID_1;
  sCommand.InstructionMode = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.Instruction = 0xC0;
  sCommand.Address = 0x00040000;
  sCommand.AddressMode = HAL_OSPI_ADDRESS_8_LINES;
  sCommand.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_ENABLE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = HAL_OSPI_DATA_8_LINES;
  sCommand.DataDtrMode = HAL_OSPI_DATA_DTR_ENABLE;
  sCommand.DummyCycles = DUMMY_CYCLES;
  sCommand.DQSMode = HAL_OSPI_DQS_ENABLE;
  sCommand.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
  sCommand.NbData = 0x4;

  if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    bsp_reset();
  }

  uint8_t dataRead[0x4] = {0xF};

  if (HAL_OSPI_Receive(hospi, dataRead, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    bsp_reset();
  }

  reg = (dataRead[3] << 8) + dataRead[2];

  return reg;
}

static void BSP_START_TEXT_SECTION ram_config_register_write(OSPI_HandleTypeDef *hospi, uint16_t val)
{
  OSPI_RegularCmdTypeDef   sCommand = {0};

  /* Initialize the write command */
  sCommand.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
  sCommand.FlashId = HAL_OSPI_FLASH_ID_1;
  sCommand.InstructionMode = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.Instruction = 0x40;
  sCommand.Address = 0x00040000;
  sCommand.AddressMode = HAL_OSPI_ADDRESS_8_LINES;
  sCommand.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_ENABLE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode = HAL_OSPI_DATA_8_LINES;
  sCommand.DataDtrMode = HAL_OSPI_DATA_DTR_ENABLE;
  sCommand.DummyCycles = 0;
  sCommand.DQSMode = HAL_OSPI_DQS_DISABLE;
  sCommand.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
  sCommand.NbData = 0x2;

  if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    bsp_reset();
  }

  uint8_t dataToWrite[2] = {val & 0xFF, (val >> 8) & 0xFF};

  if (HAL_OSPI_Transmit(hospi, dataToWrite, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
    bsp_reset();
  }
}

void BSP_START_TEXT_SECTION stm32u5_init_octospi(void)
{
  OSPI_HandleTypeDef hospi1 = {0};
  OSPIM_CfgTypeDef sOspiManagerCfg = {0};
  HAL_OSPI_DLYB_CfgTypeDef HAL_OSPI_DLYB_Cfg_Struct = {0};

  sOspiManagerCfg.ClkPort = 1;
  sOspiManagerCfg.DQSPort = 1;
  sOspiManagerCfg.NCSPort = 1;
  sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  sOspiManagerCfg.IOHighPort = HAL_OSPIM_IOPORT_1_HIGH;
  if (HAL_OSPIM_Config(&hospi1, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    bsp_reset();
  }

  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX_RAM;
  hospi1.Init.DeviceSize = 25;
  hospi1.Init.ChipSelectHighTime = 1; /* min 6ns -> 1 works for up to 160MHz */
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED; // FIXME check for performance reasons
  hospi1.Init.ClockPrescaler = 2;
  /*
   * Reference Manual: The firmware must clear SSHIFT when the data phase is
   * configured in DTR mode (DDTR = 1).
   */
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  /*
   * Reference Manual: In DTR mode, it is recommended to set DHQC of
   * OCTOSPI_TCR, to shift the outputs by a quarter of cycle and avoid holding
   * issues on the memory side.
   */
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
  hospi1.Init.ChipSelectBoundary = 7;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_USED;
  hospi1.Init.MaxTran = 0;
  hospi1.Init.Refresh = 241; // started w/ 0. changed based on AN5050
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    bsp_reset();
  }

  /* Delay block tuning */
  if (HAL_OSPI_DLYB_GetClockPeriod(&hospi1, &HAL_OSPI_DLYB_Cfg_Struct) != HAL_OK)
  {
    bsp_reset();
  }
  /* From ST example: Set at one quarter. */
  HAL_OSPI_DLYB_Cfg_Struct.PhaseSel /= 4;
  if (HAL_OSPI_DLYB_SetConfig(&hospi1, &HAL_OSPI_DLYB_Cfg_Struct) != HAL_OK)
  {
    bsp_reset();
  }

  uint16_t control_reg;
  control_reg = ram_config_register_read(&hospi1);
  ram_config_register_write(&hospi1, RAM_CONFIG_REGISTER);
  control_reg = ram_config_register_read(&hospi1);
  if ((control_reg & ~IS66_CFG_RESERVED_MASK) != RAM_CONFIG_REGISTER)
  {
    bsp_reset();
  }

  OSPI_RegularCmdTypeDef   sCommand = {0};
  OSPI_MemoryMappedTypeDef sMemMappedCfg = {0};

  /* Initialize the write command */
  sCommand.OperationType      = HAL_OSPI_OPTYPE_WRITE_CFG;
  sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
  sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_8_LINES;
  sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  sCommand.Instruction        = 0x20;
  sCommand.Address            = 0x00000000;
  sCommand.AddressMode        = HAL_OSPI_ADDRESS_8_LINES;
  sCommand.AddressSize        = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_ENABLE;
  sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = HAL_OSPI_DATA_8_LINES;
  sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_ENABLE;
  sCommand.DummyCycles        = DUMMY_CYCLES;
  sCommand.DQSMode            = HAL_OSPI_DQS_ENABLE;
  sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;
  sCommand.NbData             = 0x1; // had 0. changed based on AN5050

  if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    bsp_reset();
  }

  sCommand.OperationType = HAL_OSPI_OPTYPE_READ_CFG;
  sCommand.Instruction = 0xA0;

  if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
      bsp_reset();
  }

  /* OctoSPI activation of memory-mapped mode */
  sMemMappedCfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE; //had enable. changed based on AN5050
  //sMemMappedCfg.TimeOutPeriod     = 0x34U; // had this, changed based on AN5050

  if (HAL_OSPI_MemoryMapped(&hospi1, &sMemMappedCfg) != HAL_OK)
  {
    bsp_reset();
  }

}

void BSP_START_TEXT_SECTION HAL_OSPI_MspInit(OSPI_HandleTypeDef *hospi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(hospi->Instance==OCTOSPI1)
  {
    /* Initializes the peripherals clock */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_OSPI;
    PeriphClkInit.OspiClockSelection = RCC_OSPICLKSOURCE_PLL2;
    PeriphClkInit.PLL2.PLL2Source = RCC_PLLSOURCE_MSI;
    PeriphClkInit.PLL2.PLL2M = 3;
    PeriphClkInit.PLL2.PLL2N = 12;
    PeriphClkInit.PLL2.PLL2P = 2;
    PeriphClkInit.PLL2.PLL2Q = 1;
    PeriphClkInit.PLL2.PLL2R = 2;
    PeriphClkInit.PLL2.PLL2RGE = RCC_PLLVCIRANGE_1;
    PeriphClkInit.PLL2.PLL2FRACN = 4096.0;
    PeriphClkInit.PLL2.PLL2ClockOut = RCC_PLL2_DIVQ;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      bsp_reset();
    }

    /* Peripheral clock enable */
    __HAL_RCC_OSPIM_CLK_ENABLE();
    __HAL_RCC_OSPI1_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* OCTOSPI1 GPIO Configuration */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPI1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPI1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPI1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
}
