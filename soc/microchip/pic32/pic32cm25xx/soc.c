#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>

#include <soc.h>

void cpuio_init(void)
{
	CPUIO_C0_CTRL_REGS->CPUIO_C0_CTRL_C0_APBA_ACCESS = 0xFFFFFFFF;
	CPUIO_C0_CTRL_REGS->CPUIO_C0_CTRL_C0_APBB_ACCESS = 0xFFFFFFFF;
	CPUIO_C0_CTRL_REGS->CPUIO_C0_CTRL_C1_APBA_ACCESS = 0xFFFFFFFF;
	CPUIO_C0_CTRL_REGS->CPUIO_C0_CTRL_C1_APBB_ACCESS = 0xFFFFFFFF;
	CPUIO_C0_CTRL_REGS->CPUIO_C0_CTRL_DMACR_APBA_ACCESS = 0xFFFFFFFF;
	CPUIO_C0_CTRL_REGS->CPUIO_C0_CTRL_DMACR_APBB_ACCESS = 0xFFFFFFFF;
	CPUIO_C0_CTRL_REGS->CPUIO_C0_CTRL_DMACW_APBA_ACCESS = 0xFFFFFFFF;
	CPUIO_C0_CTRL_REGS->CPUIO_C0_CTRL_DMACW_APBB_ACCESS = 0xFFFFFFFF;
	CPUIO_C0_CTRL_REGS->CPUIO_C0_CTRL_OASPI_APBB_ACCESS = 0xFFFFFFFF;

	/* Disable 2nd core */
	CPUIO_C0_CTRL_REGS->CPUIO_C0_CTRL_C1_CONTROL |= CPUIO_C0_CTRL_C1_CONTROL_CPUWAIT_Msk;
	CPUIO_C0_CTRL_REGS->CPUIO_C0_CTRL_C1_CONTROL |= CPUIO_C0_CTRL_C1_CONTROL_SWRST_Msk;
}

void pll_init(void)
{
	/* Enable APLL, switch to 50MHz system clock and wait for the APLL to lock */
	APLL_REGS->APLL_CTRL &= ~APLL_CTRL_PLLPD_Msk;
	CLK_RST_GEN_REGS->CLK_RST_GEN_CG_SRC_CTRL2 &= ~CLK_RST_GEN_CG_SRC_CTRL2_SWITCH_SEL_Msk;
	while (!(APLL_REGS->APLL_INT_STATUS & APLL_INT_STATUS_LOCK_Msk)) {
	}
}

void flash_init(void)
{
	/* Configure FLASH
	 * - Enable cache, prefetch optimization and prog loop optimization
	 * - Set wait states for main and signature FLASH to 1
	 */
	FLASHC_REGS->FLASHC_FC_FMR |= FLASHC_FC_FMR_FLASH_CACHE_ENABLE(1) |
				      FLASHC_FC_FMR_SEQ_PREFETCH_OPT(1) |
				      FLASHC_FC_FMR_PROG_LOOP_OPT (1);
	FLASHC_REGS->FLASHC_FC_FMR |= FLASHC_FC_FMR_FLASH_WAIT_STATE(1);
	FLASHC_REGS->FLASHC_FC_FMR |= FLASHC_FC_FMR_FLASH_SIGNATURE_WAIT_STATES(1);
}

void clock_configure(void)
{
	/* Enable all clocks */
	CLK_RST_GEN_REGS->CLK_RST_GEN_CG_SYS_MASK0 = 0xFFFFFFFF;
	CLK_RST_GEN_REGS->CLK_RST_GEN_CG_SYS_MASK1 = 0xFFFFFFFF;
	CLK_RST_GEN_REGS->CLK_RST_GEN_CG_SYS_MASK2 = 0xFFFFFFFF;
	CLK_RST_GEN_REGS->CLK_RST_GEN_CG_SRC_MASK = 0xFFFFFFFF;
}

void test_mode_controller_init(void)
{
	/* Run the MBIST test to clear ECC errors */
	/* !!! Until this code is reached, no SRAM accesses should must be done, including no stack usage. !!! */
	uint32_t start_mask = TMODEC_SRAM_MBIST_STATUS_SRAM0_BIST_START_Msk |
			      TMODEC_SRAM_MBIST_STATUS_SRAM1_BIST_START_Msk |
			      TMODEC_SRAM_MBIST_STATUS_SRAM2_BIST_START_Msk |
			      TMODEC_SRAM_MBIST_STATUS_SRAM3_BIST_START_Msk;
	uint32_t done_mask = TMODEC_SRAM_MBIST_STATUS_SRAM0_BIST_DONE_Msk |
			     TMODEC_SRAM_MBIST_STATUS_SRAM1_BIST_DONE_Msk |
			     TMODEC_SRAM_MBIST_STATUS_SRAM2_BIST_DONE_Msk |
			     TMODEC_SRAM_MBIST_STATUS_SRAM3_BIST_DONE_Msk;

	TMODEC_REGS->TMODEC_SRAM_MBIST_STATUS = start_mask;
	while ((TMODEC_REGS->TMODEC_SRAM_MBIST_STATUS & done_mask) != done_mask) {
	}
}

void z_arm_platform_init(void)
{
	cpuio_init();
	pll_init();
	flash_init();
	clock_configure();
	//test_mode_controller_init();
}
