/*
 * Driver for TS-7800-V2's FPGA based RNG.  This outputs random data 
 * based on a core that outputs continuous data based on metastability.
 * 
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/hw_random.h>
#include <linux/pci.h>

#include <asm/io.h>

static int ts7800v2_rng_data_read(struct hwrng *rng, u32 *buffer)
{
	void __iomem * rng_base = (void __iomem *)rng->priv;
	*buffer = readl(rng_base + 0x44);
	return 4;
}

static struct hwrng ts7800v2_rng_ops = {
	.name		= "ts7800v2",
	.data_read	= ts7800v2_rng_data_read,
};

static int __init ts7800v2_rng_init(void)
{
	void __iomem *rng_base;
	struct pci_dev *pcidev;
	int err;

	/* The pcie must be enabled before we can access the fpga registers! */
	pcidev = pci_get_device(0x1204, 0x0001, NULL);
	if (!pcidev) {
		pr_err("Cannot find FPGA at PCI 1204:0001\n");
		return -EINVAL;
	}

	if (pci_enable_device(pcidev)) {
		pr_err("Cannot enable FPGA at PCI 1204:0001\n");
		return -EINVAL;
	}

	rng_base = ioremap_nocache(0xFC081000, 4);
	if (!rng_base)
		return -ENOMEM;

	ts7800v2_rng_ops.priv = (unsigned long)rng_base;
	err = hwrng_register(&ts7800v2_rng_ops);
	if (err)
		iounmap(rng_base);

	return err;
}

static void __exit ts7800v2_rng_exit(void)
{
	void __iomem * rng_base = (void __iomem *)ts7800v2_rng_ops.priv;

	hwrng_unregister(&ts7800v2_rng_ops);
	iounmap(rng_base);
}

module_init(ts7800v2_rng_init);
module_exit(ts7800v2_rng_exit);

MODULE_AUTHOR("Mark Featherston <mark@embeddedarm.com>");
MODULE_DESCRIPTION("FPGA Based RNG");
MODULE_LICENSE("GPL");
