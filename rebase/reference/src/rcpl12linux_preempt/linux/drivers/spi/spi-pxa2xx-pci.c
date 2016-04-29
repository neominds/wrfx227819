/*
 * CE4100's SPI device is more or less the same one as found on PXA
 *
 */
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/clk.h>

static bool use_dma = 1;
module_param(use_dma, bool, 0);
MODULE_PARM_DESC(use_dma, "use_dma=1 (enable DMA)/use_dma=0(disable DMA)");

enum {
	PORT_CE4100,
	PORT_BYT,
};

struct pxa2xx_spi_pci_config {
	enum pxa_ssp_type type;
	int num_cs;
	int bus_num;
	int tx_slave_id;
	int tx_chan_id;
	int rx_slave_id;
	int rx_chan_id;
};

static struct pxa2xx_spi_pci_config spi_pci_configs[] = {
	[PORT_CE4100] = {
		.type = PXA25x_SSP,
		.num_cs =  -1,
		.bus_num = -1,
		.tx_slave_id = -1,
		.tx_chan_id = -1,
		.rx_slave_id = -1,
		.rx_chan_id = -1,
	},
	[PORT_BYT] = {
		.type = LPSS_SSP,
		.num_cs = 1,
		.bus_num = 0,
		.tx_slave_id = 0,
		.tx_chan_id = 0,
		.rx_slave_id = 1,
		.rx_chan_id = 1,
	},
};

static int pxa2xx_spi_probe(struct pci_dev *dev,
		const struct pci_device_id *ent)
{
	struct pxa2xx_spi_pci_config *c;
	struct platform_device_info pi;
	int ret;
	struct platform_device *pdev;
	struct pxa2xx_spi_master spi_pdata;
	struct ssp_device *ssp;

	ret = pcim_enable_device(dev);
	if (ret)
		return ret;

	ret = pcim_iomap_regions(dev, 1 << 0, "PXA2xx SPI");
	if (ret)
		return ret;

	c = &spi_pci_configs[ent->driver_data];

	memset(&spi_pdata, 0, sizeof(spi_pdata));
	spi_pdata.num_chipselect = (c->num_cs >= 0) ? c->num_cs : dev->devfn;
//	spi_pdata.num_chipselect = dev->devfn;

	spi_pdata.tx_slave_id = c->tx_slave_id;
	spi_pdata.tx_chan_id = c->tx_chan_id;
	spi_pdata.rx_slave_id = c->rx_slave_id;
	spi_pdata.rx_chan_id = c->rx_chan_id;
	if (use_dma)
	       	spi_pdata.enable_dma = c->rx_slave_id >= 0 && c->tx_slave_id >= 0;

	ssp = &spi_pdata.ssp;
	ssp->phys_base = pci_resource_start(dev, 0);
	ssp->mmio_base = pcim_iomap_table(dev)[0];
	if (!ssp->mmio_base) {
		dev_err(&dev->dev, "failed to ioremap() registers\n");
		return -EIO;
	}
	ssp->clk =  devm_clk_get(&dev->dev, NULL);
	ssp->irq = dev->irq;
	ssp->port_id = (c->bus_num >= 0) ? c->bus_num : dev->devfn;
	ssp->type = c->type;

	memset(&pi, 0, sizeof(pi));
	pi.parent = &dev->dev;
	pi.name = "pxa2xx-spi";
	pi.id = ssp->port_id;
	pi.data = &spi_pdata;
	pi.size_data = sizeof(spi_pdata);

	pdev = platform_device_register_full(&pi);
	if (IS_ERR(pdev))
		return PTR_ERR(pdev);

	pci_set_drvdata(dev, pdev);

	return 0;
}

static void pxa2xx_spi_remove(struct pci_dev *dev)
{
	struct platform_device *pdev = pci_get_drvdata(dev);

	platform_device_unregister(pdev);
}

static DEFINE_PCI_DEVICE_TABLE(pxa2xx_spi_devices) = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x2e6a),
	  .driver_data =  PORT_CE4100 },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x0f0e),
	  .driver_data = PORT_BYT },
	{ },
};
MODULE_DEVICE_TABLE(pci, pxa2xx_spi_devices);

static struct pci_driver pxa2xx_spi_driver = {
	.name           = "pxa2xx_spi",
	.id_table       = pxa2xx_spi_devices,
	.probe          = pxa2xx_spi_probe,
	.remove         = pxa2xx_spi_remove,
};

module_pci_driver(pxa2xx_spi_driver);

MODULE_DESCRIPTION("PCI-SPI glue code for PXA's driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sebastian Andrzej Siewior <bigeasy@linutronix.de>");
