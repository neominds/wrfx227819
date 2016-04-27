#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/spi/spidev.h>
#include <linux/spi/spi.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/pwm.h>

static bool disable = 0;
module_param(disable, bool, 0);
MODULE_PARM_DESC(disable, "set 1 to disable BYT brd file");

static struct pxa2xx_spi_chip chip_data = {
	.gpio_cs = -EINVAL,
	.dma_burst_size = 32,
};

static struct spi_board_info byt_spi_slaves[] = {
	{
	 .modalias = "spidev",
	 .max_speed_hz = 50000000,
	 .bus_num = 0,
	 .chip_select = 0,
	 .controller_data = &chip_data,
	 .mode = SPI_MODE_0,
	}
};

static int byt_spi_board_setup(void)
{
	int ret = -1;

	/* Register the SPI devices */
	if (!spi_register_board_info
		(byt_spi_slaves, ARRAY_SIZE(byt_spi_slaves)))
		ret = 0;

	return ret;
}

static int byt_clk_setup(void)
{
	struct clk *clk;

	/* Make sure the root clk required by the LPSS driver is registered */
	clk = clk_get(NULL, "lpss_clk");
	if (IS_ERR(clk)) {
		clk = clk_register_fixed_rate(NULL, "lpss_clk", NULL, CLK_IS_ROOT,
							100000000);
	}

	/* 
	 * to check has the spi_clk been registered by the ACPI mode,
	 * if yes, skip it, otherwise, register those clks.
	*/
	clk = clk_get(NULL, "spi_clk");
	if (IS_ERR(clk)) {
		clk = clk_register_fixed_rate(NULL, "spi_clk", "lpss_clk", 0, 50000000);
		if (IS_ERR(clk))
			return PTR_ERR(clk);
	} else
		return PTR_ERR(clk);

	clk_register_clkdev(clk, NULL, "0000:00:1e.5");

	/* register the clock for DW DMA engines */
	clk_register_clkdev(clk, "hclk", "0000:00:18.0");
	clk_register_clkdev(clk, "hclk", "0000:00:1e.0");

	/* Make clock tree required by the PWM driver */
	clk = clk_register_fixed_rate(NULL, "pwm_clk", "lpss_clk", 0, 25000000);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	clk_register_clkdev(clk, NULL, "0000:00:1e.1");
	clk_register_clkdev(clk, NULL, "0000:00:1e.2");

	return 0;
}

static int __init byt_board_init(void)
{
	int ret;

	if (disable)
		return 0;

	ret = byt_clk_setup();
	if (ret)
		goto exit;

	ret = byt_spi_board_setup();
	if (ret)
		goto exit;

exit:
	return ret;
}

/* make sure the board init function was called after acpi init. */
subsys_initcall_sync(byt_board_init);
MODULE_LICENSE(GPL);
