/*
 *  Setup code for Goodwin boards based on at91sam9g20 with additional
 *  Ethernet MAC (ax88796b/c) on CPU bus.
 *
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/delay.h>

#include <asm/setup.h>
#include <asm/irq.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/gpio.h>
#include <mach/at91sam9_smc.h>
#include "sam9_smc.h"

#include "at91_aic.h"
#include "board.h"
#include "generic.h"


static const struct of_device_id irq_of_match[] __initconst = {

	{ .compatible = "atmel,at91rm9200-aic", .data = at91_aic_of_init },
	{ /*sentinel*/ }
};

static void __init at91_dt_init_irq(void)
{
	of_irq_init(irq_of_match);
}

static struct sam9_smc_config __initdata ax88796c_smc_config = {
	.ncs_read_setup  = 0,
	.nrd_setup       = 1, // ax=0ns, at=0?
	.ncs_write_setup = 0,
	.nwe_setup       = 1,

	.ncs_read_pulse  = 8, // nrd_setup + nrd_pulse +1
	.nrd_pulse       = 6, // ax=35ns
	.ncs_write_pulse = 8,
	.nwe_pulse       = 6,

	.read_cycle      = 14,
	.write_cycle     = 14,

	.tdf_cycles      = 3, // ax=7ns -> 1

	.mode = AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_DBW_16 \
	      | AT91_SMC_EXNWMODE_DISABLE | AT91_SMC_BAT_WRITE,
};

static void __init at91_dt_device_init(void)
{
	at91sam9_ioremap_smc(0, AT91SAM9260_BASE_SMC);
	sam9_smc_configure(0, 0, &ax88796c_smc_config);

	at91_set_GPIO_periph(AT91_PIN_PC3, 0);
	// reset ax88796, 200us min
	at91_set_gpio_output(AT91_PIN_PC3, 0);
	msleep(500);
	at91_set_gpio_value(AT91_PIN_PC3, 1);
	
	at91_set_A_periph(AT91_PIN_PC12, 1); // IRQ0

	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

static const char *at91_dt_board_compat[] __initdata = {
	"goodwin,at91sam9-extmac",
	NULL
};

DT_MACHINE_START(goodwin_ext_mac_dt,
		 "Goodwin AT91SAM9 with external Ethernet (Device Tree)")
	/* Maintainer: Goodwin */
	.init_time	= at91sam926x_pit_init,
	.map_io		= at91_map_io,
	.handle_irq	= at91_aic_handle_irq,
	.init_early	= at91_dt_initialize,
	.init_irq	= at91_dt_init_irq,
	.init_machine	= at91_dt_device_init,
	.dt_compat	= at91_dt_board_compat,
MACHINE_END
