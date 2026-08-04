/*
 * Configure the fixed CPU operating point used by GRiSP2.
 *
 * The RTEMS 5 i.MX BSP has no cpufreq framework and therefore otherwise
 * inherits the 396 MHz operating point left by barebox.
 */

#include <stdint.h>

#include <libfdt.h>

#include "imx6ull-clock.h"

#define IMX6_CCM_BASE                    0x020c4000U
#define IMX6_CCM_CACRR                   0x00000010U
#define IMX6_CCM_CDHIPR                  0x00000048U
#define IMX6_CCM_CACRR_ARM_PODF_MASK     0x00000007U
#define IMX6_CCM_CACRR_ARM_PODF_DIV_2    0x00000001U
#define IMX6_CCM_CDHIPR_ARM_PODF_BUSY    0x00010000U

#define IMX6_ANATOP_BASE                 0x020c8000U
#define IMX6_ANATOP_PLL_ARM              0x00000000U
#define IMX6_ANATOP_PLL_ARM_DIV_MASK     0x0000007fU
#define IMX6_ANATOP_PLL_ARM_DIV_792_MHZ  66U
#define IMX6_ANATOP_PLL_ARM_POWERDOWN    0x00001000U
#define IMX6_ANATOP_PLL_ARM_ENABLE       0x00002000U
#define IMX6_ANATOP_PLL_ARM_BYPASS       0x00010000U
#define IMX6_ANATOP_PLL_ARM_LOCK         0x80000000U
#define IMX6_ANATOP_REG_CORE             0x00000140U
#define IMX6_ANATOP_REG0_TRG_MASK        0x0000001fU
#define IMX6_ANATOP_REG2_TRG_SHIFT       18U
#define IMX6_ANATOP_REG2_TRG_MASK        (0x1fU << IMX6_ANATOP_REG2_TRG_SHIFT)

#define IMX6_OCOTP_BASE                  0x021bc000U
#define IMX6_OCOTP_CFG3                  0x00000440U
#define IMX6_OCOTP_SPEED_GRADE_SHIFT     16U
#define IMX6_OCOTP_SPEED_GRADE_MASK      0x3U
#define IMX6ULL_SPEED_GRADE_792_MHZ      0x2U

/* The selectors use 25 mV steps, with selector 1 corresponding to 725 mV. */
#define IMX6ULL_VDD_ARM_1225_MV_SELECTOR 21U
#define IMX6ULL_VDD_SOC_1175_MV_SELECTOR 19U

#define IMX6ULL_VOLTAGE_SETTLE_LOOPS     400000U
#define IMX6ULL_CCM_HANDSHAKE_LOOPS      1000000U

static volatile uint32_t *imx6_reg(uintptr_t base, uint32_t offset)
{
  return (volatile uint32_t *)(base + offset);
}

static void imx6ull_voltage_settle(void)
{
  uint32_t i;

  for (i = 0; i < IMX6ULL_VOLTAGE_SETTLE_LOOPS; ++i) {
    __asm__ volatile ("nop");
  }
}

static void imx6ull_raise_voltage(
  uint32_t mask,
  uint32_t shift,
  uint32_t selector
)
{
  volatile uint32_t *reg;
  uint32_t value;
  uint32_t current;

  reg = imx6_reg(IMX6_ANATOP_BASE, IMX6_ANATOP_REG_CORE);
  value = *reg;
  current = (value & mask) >> shift;

  if (current < selector) {
    value = (value & ~mask) | (selector << shift);
    *reg = value;
    imx6ull_voltage_settle();
  }
}

static int imx6ull_pll_is_792_mhz(void)
{
  uint32_t pll;

  pll = *imx6_reg(IMX6_ANATOP_BASE, IMX6_ANATOP_PLL_ARM);

  return
    (pll & IMX6_ANATOP_PLL_ARM_DIV_MASK) ==
      IMX6_ANATOP_PLL_ARM_DIV_792_MHZ &&
    (pll & IMX6_ANATOP_PLL_ARM_POWERDOWN) == 0U &&
    (pll & IMX6_ANATOP_PLL_ARM_ENABLE) != 0U &&
    (pll & IMX6_ANATOP_PLL_ARM_BYPASS) == 0U &&
    (pll & IMX6_ANATOP_PLL_ARM_LOCK) != 0U;
}

void imx6ull_setup_cpu_frequency(const void *fdt)
{
  volatile uint32_t *cacrr;
  volatile uint32_t *cdhipr;
  uint32_t speed_grade;
  uint32_t value;
  uint32_t timeout;

  if (fdt_node_check_compatible(fdt, 0, "embeddedbrains,grisp2") != 0 ||
      fdt_node_check_compatible(fdt, 0, "fsl,imx6ull") != 0) {
    return;
  }

  value = *imx6_reg(IMX6_OCOTP_BASE, IMX6_OCOTP_CFG3);
  speed_grade =
    (value >> IMX6_OCOTP_SPEED_GRADE_SHIFT) &
    IMX6_OCOTP_SPEED_GRADE_MASK;
  if (speed_grade < IMX6ULL_SPEED_GRADE_792_MHZ ||
      !imx6ull_pll_is_792_mhz()) {
    return;
  }

  cacrr = imx6_reg(IMX6_CCM_BASE, IMX6_CCM_CACRR);
  value = *cacrr;
  if ((value & IMX6_CCM_CACRR_ARM_PODF_MASK) == 0U) {
    return;
  }
  if ((value & IMX6_CCM_CACRR_ARM_PODF_MASK) !=
      IMX6_CCM_CACRR_ARM_PODF_DIV_2) {
    return;
  }

  /* On frequency increases, raise the SoC and ARM rails before the clock. */
  imx6ull_raise_voltage(
    IMX6_ANATOP_REG2_TRG_MASK,
    IMX6_ANATOP_REG2_TRG_SHIFT,
    IMX6ULL_VDD_SOC_1175_MV_SELECTOR
  );
  imx6ull_raise_voltage(
    IMX6_ANATOP_REG0_TRG_MASK,
    0U,
    IMX6ULL_VDD_ARM_1225_MV_SELECTOR
  );

  __asm__ volatile ("dsb sy" ::: "memory");
  *cacrr = value & ~IMX6_CCM_CACRR_ARM_PODF_MASK;
  __asm__ volatile ("dsb sy" ::: "memory");

  cdhipr = imx6_reg(IMX6_CCM_BASE, IMX6_CCM_CDHIPR);
  timeout = IMX6ULL_CCM_HANDSHAKE_LOOPS;
  while ((*cdhipr & IMX6_CCM_CDHIPR_ARM_PODF_BUSY) != 0U && timeout != 0U) {
    --timeout;
  }

  __asm__ volatile ("isb" ::: "memory");
}
