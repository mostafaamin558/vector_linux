/*
 * linux/arch/arm/mach-omap2/board-varam35.c
 *
 * Board support for HIKOB / Variscite platform
 *
 * Copyright(C) 2013,2014 HiKoB SA.
 * Author: Antoine Fraboulet <antoine.fraboulet.at.hikob.com>
 * HiKoB specific board support :
 *    - Multimedia / Display disabled
 *    - USB OTG disabled
 *    - low power configuration
 *
 * Based on mach-omap2/board-am3517evm.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as  published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/platform_data/pca953x.h>
#include <linux/can/platform/ti_hecc.h>
#include <linux/davinci_emac.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mmc/host.h>
#include <linux/usb/musb.h>
#include <linux/platform_data/gpio-omap.h>
#include <linux/platform_data/mtd-nand-omap2.h>

#include "am35xx.h"
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include "common.h"
#include <video/omapdss.h>
#include <video/omap-panel-data.h>

#include "am35xx-emac.h"
#include "mux.h"
#include "control.h"
#include "gpmc.h"
#include "hsmmc.h"
#include "board-flash.h"


static struct i2c_board_info __initdata varam35_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("ds1307", 0x68),
		.type = "ds1307",
	},
};


static int __init varam35_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, NULL, 0);
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, NULL, 0);

	return 0;
}


/*
 * NAND
 */

#define NAND_BLOCK_SIZE        SZ_128K

/*
 * Nand permissions
 *     .mask_flags: removes permissions
 */
#ifdef CONFIG_VARAM35_MTD_WRITEABLE
#define VARAM35_NAND_MODE_MASK 0
#else
#define VARAM35_NAND_MODE_MASK MTD_WRITEABLE
#endif

static struct mtd_partition varam35_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,                    /* offset : O        */
		.size		= 4*(NAND_BLOCK_SIZE),  /* size   : 512kB    */
		.mask_flags	= VARAM35_NAND_MODE_MASK,
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,   /* offset : 0x80000  */
		.size		= 14*(NAND_BLOCK_SIZE), /* size   : 0x1c0000 */
		.mask_flags	= VARAM35_NAND_MODE_MASK,
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,   /* offset : 0x240000 */
		.size		= 2*(NAND_BLOCK_SIZE),  /* size   : 0x040000 */
                .mask_flags	= VARAM35_NAND_MODE_MASK,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,   /* offset : 0x280000 */
		.size		= 40*(NAND_BLOCK_SIZE), /* size   : 0x500000 */
                .mask_flags	= VARAM35_NAND_MODE_MASK,
	},
	{
		.name		= "File System",        /* offset : 0x780000 */
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
                .mask_flags	= VARAM35_NAND_MODE_MASK,
	},
};

/*
 * Board initialization
 */

static struct omap_musb_board_data musb_board_data = {
	.interface_type         = MUSB_INTERFACE_ULPI,
	.mode                   = MUSB_OTG,
	.power                  = 500,
	.set_phy_power		= am35x_musb_phy_power,
	.clear_irq		= am35x_musb_clear_irq,
	.set_mode		= am35x_set_mode,
	.reset			= am35x_musb_reset,
};

static __init void varam35_musb_init(void)
{
	u32 devconf2;

	/*
	 * Set up USB clock/mode in the DEVCONF2 register.
	 */
	devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

	/* USB2.0 PHY reference clock is 13 MHz */
	devconf2 &= ~(CONF2_REFFREQ | CONF2_OTGMODE | CONF2_PHY_GPIOMODE);
	devconf2 |=  CONF2_REFFREQ_13MHZ | CONF2_SESENDEN | CONF2_VBDTCTEN
			| CONF2_DATPOL;

	omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);

	usb_musb_init(&musb_board_data);
}

/* McBSP :: Multi Channel Buffered Serial Port */
static __init void varam35_mcbsp1_init(void)
{
	u32 devconf0;

	/* McBSP1 CLKR/FSR signal to be connected to CLKX/FSX pin */
	devconf0 = omap_ctrl_readl(OMAP2_CONTROL_DEVCONF0);
	devconf0 |=  OMAP2_MCBSP1_CLKR_MASK | OMAP2_MCBSP1_FSR_MASK;
	omap_ctrl_writel(devconf0, OMAP2_CONTROL_DEVCONF0);
}

/*
 * USB
 */

static struct usbhs_phy_data phy_data[] __initdata = {
	{
		.port = 1,
		.reset_gpio = 154,
		.vcc_gpio = -EINVAL,
	},
	{
		.port = 2,
		.reset_gpio = 152,
		.vcc_gpio = -EINVAL,
	},
};

static struct usbhs_omap_platform_data usbhs_bdata __initdata = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_EHCI_PORT_MODE_PHY,
	//.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
};

/*
 * MUX
 */

static struct omap_board_mux board_mux[] __initdata = {
	OMAP3_MUX(CHASSIS_DMAREQ3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP3_MUX(SDMMC2_CMD,  OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_CLK,  OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT0, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT1, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT2, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT3, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT4, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT5, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT6, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP3_MUX(SDMMC2_DAT7, OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

#define SD_CARD_CD		61     // MMC1_CD (OUT)
#define SD_CARD_WP		65     // SD_WP   (OUT)

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= SD_CARD_CD,
		.gpio_wp	= SD_CARD_WP,
	},
	{
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{}      /* Terminator */
};

static void __init varam35_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);

	varam35_i2c_init();

	omap_serial_init();
	omap_sdrc_init(NULL, NULL);


	/* Configure GPIO for EHCI port */
	omap_mux_init_gpio(57, OMAP_PIN_OUTPUT);

	usbhs_init_phys(phy_data, ARRAY_SIZE(phy_data));
	usbhs_init(&usbhs_bdata);

	/* Nand Flash Storage */
	board_nand_init(varam35_nand_partitions,
			ARRAY_SIZE(varam35_nand_partitions), 0,
			NAND_BUSWIDTH_16, NULL);

	/* i2c */
	i2c_register_board_info(1, varam35_i2c1_boardinfo,
				ARRAY_SIZE(varam35_i2c1_boardinfo));

	/*Ethernet*/
	am35xx_emac_init(AM35XX_DEFAULT_MDIO_FREQUENCY, 1);

	/* MUSB */
	varam35_musb_init();

	/* McBSP1 serial port */
	varam35_mcbsp1_init();

	/* SD/eMMC init function */
	omap_mux_init_gpio(SD_CARD_CD, OMAP_PIN_INPUT);
	omap_mux_init_gpio(SD_CARD_WP, OMAP_PIN_INPUT);
	omap_hsmmc_init(mmc);

	pr_info("===\n");
	pr_info("=== Varam35 platform init done.\n");
	pr_info("===   patchlevel 5\n");
	pr_info("===\n");
}

MACHINE_START(OMAP3517EVM, "HIKOB Var-Am35")
	.atag_offset	= 0x100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= am35xx_init_early,
	.init_irq	= omap3_init_irq,
	.init_machine	= varam35_init,
	.init_late	= am35xx_init_late,
	.init_time	= omap3_sync32k_timer_init,
	.restart	= omap3xxx_restart,
MACHINE_END
