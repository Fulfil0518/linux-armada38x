/*
 * ts7800v2_can.c - Interface for the CAN controller in the TS-7800v2's FPGA.
 *
 * Copyright (c) 2018, Technologic Systems, Inc.
 * All rights reserved.
 *
 *  LICENCE:
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/platform/sja1000.h>
#include <linux/can/skb.h>
#include <linux/slab.h>
#include <net/rtnetlink.h>

#include "sja1000.h"

#define DEBUG_ON 0
#define DEFAULT_IRQ  120

static int irq;
module_param(irq, int, S_IRUGO);
MODULE_PARM_DESC(irq, "Interrupt number.  Default: 120");


static DEFINE_SPINLOCK(fpga_lock);


/**
   There is a 32-bit register in the syscon at offset 0x4c.

   CAN has its own 256 byte 8-bit bus address space manipulated via this
   one register.

   Bit 31: Start CAN bus cycle
   Bit 30: CAN bus cycle is a write
   Bits 15-8: CAN data for read or write
   Bits 7-0: CAN address

   To initiate a CAN read for (e.g.) address 0x55, first write syscon
   register 0x4c with 0x800000aa.  Then read back the same register.  When
   bit 31 is clear, the bus cycle is complete and the data is in bits 15-8.

   To initiate a CAN write of value 0xaa for address 0x55, write
   0xc000aa55.

*/

#define DRIVER_NAME	"ts7800v2_can"
#define TX_MAX 1
#define RX_MAX 1
#define CAN_CNTL_REG_OFFSET 0x4C
#define CAN_CNTL_START BIT(31)
#define CAN_CNTL_WRITE BIT(30)


static u8 ts7800v2_read_reg(const struct sja1000_priv *priv, int reg)
{
   unsigned long flags;
   volatile unsigned int *syscon = (unsigned int *)priv->reg_base;
   unsigned int v;

   spin_lock_irqsave(&fpga_lock, flags);

   while(readl(&syscon[CAN_CNTL_REG_OFFSET / 4]) & CAN_CNTL_START);

   writel(CAN_CNTL_START | reg, &syscon[CAN_CNTL_REG_OFFSET / 4]);

   do {
      v = readl(&syscon[CAN_CNTL_REG_OFFSET / 4]);
   } while(v & CAN_CNTL_START);

#if (DEBUG_ON)
   printk("READ REG:  0x%02X == 0x%02X (raw 0x%08X)\n",
      reg, (v >> 8) & 0xFF, v);
#endif

   spin_unlock_irqrestore(&fpga_lock, flags);
   return (v >> 8) & 0xFF;
}

static void ts7800v2_write_reg(const struct sja1000_priv *priv, int reg, u8 val)
{
   unsigned long flags;
   volatile unsigned int *syscon = (unsigned int *)priv->reg_base;

#if (DEBUG_ON)
   printk("WRITE REG: 0x%02X to 0x%02X (raw 0x%08X)\n", val, reg,
      CAN_CNTL_START | CAN_CNTL_WRITE | reg | ((u16)val << 8));
#endif

   spin_lock_irqsave(&fpga_lock, flags);
   while(readl(&syscon[CAN_CNTL_REG_OFFSET / 4]) & CAN_CNTL_START);

   writel(CAN_CNTL_START | CAN_CNTL_WRITE | reg | ((u16)val << 8), &syscon[CAN_CNTL_REG_OFFSET / 4]);
   spin_unlock_irqrestore(&fpga_lock, flags);

}


static void ts7800v2_populate_of(struct sja1000_priv *priv, struct device_node *of)
{
   int err;
   u32 prop;

   err = of_property_read_u32(of, "technologic,can-clock-frequency", &prop);
   if (!err && prop) {
#if (DEBUG_ON)
      printk("%s %d, \"technologic,can-clock-frequency\" %d \n",
         __func__, __LINE__, prop);
#endif
      priv->can.clock.freq = prop / 2;

   } else {
      printk("%s: Did not find \"technologic,can-clock-frequency\" in "
            "device-tree, assuming 16MHz\n",
         __func__);

      priv->can.clock.freq = 16000000 / 2;
   }
}


/**
 * ts7800v2_can_probe - Platform registration call
 * @pdev:	Handle to the platform device structure
 *
 * This function does all the memory allocation and registration for the CAN
 * device.
 *
 * Return: 0 on success and failure value on error
 */
static int ts7800v2_can_probe(struct platform_device *pdev)
{
   struct resource *res_mem; /* IO mem resources */
   struct net_device *ndev;
   struct sja1000_priv *priv;
   void __iomem *addr;
   volatile unsigned int *syscon;
   unsigned long mem_size;
   int ret, fpga_rev;
   struct device_node *of = pdev->dev.of_node;

#if (DEBUG_ON)
   printk("%s %d\n", __func__, __LINE__);
#endif

   if (!of) {
        dev_err(&pdev->dev, "no node for ts7800v2_can was found in Device-Tree\n");
        return -ENODEV;
   }

    /* Get the virtual base address for the device */
   res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   if (!res_mem) {
      dev_err(&pdev->dev, "no MEM specified in Device-Tree\n");
      return -ENXIO;
   }

   mem_size = resource_size(res_mem);
   addr = devm_ioremap_nocache(&pdev->dev, res_mem->start, resource_size(res_mem));

   if (IS_ERR(addr)) {
      ret = PTR_ERR(addr);
      return -ENOMEM;
   }

   if (!irq)
      irq = DEFAULT_IRQ;

   syscon = (unsigned int *)addr;
   fpga_rev = readl(syscon) & 0xFF;

   if (fpga_rev < 39) {
      dev_err(&pdev->dev, "FPGA needs to be Rev 39 or later. Found Rev %d\n", fpga_rev);
      return -ENODEV;
   }

   ndev = alloc_sja1000dev(0);
   if (!ndev)
      return -ENOMEM;

   priv = netdev_priv(ndev);
   ndev->irq = irq;
   priv->irq_flags = IRQF_SHARED;
   priv->reg_base = addr;
   priv->ocr = OCR_TX0_PULLUP | OCR_TX1_PULLUP | OCR_MODE_NORMAL;
   priv->cdr = CDR_CBP | CDR_PELICAN; /* comparator bypass, pelican mode */
   ts7800v2_populate_of(priv, of);

   priv->read_reg = ts7800v2_read_reg;
   priv->write_reg = ts7800v2_write_reg;

   ts7800v2_write_reg(priv, SJA1000_MOD, 1);     /* Enter reset mode */
   ts7800v2_write_reg(priv, SJA1000_CDR, priv->cdr);
   ts7800v2_write_reg(priv, SJA1000_OCR,  priv->ocr);
   ts7800v2_write_reg(priv, SJA1000_BTR0, 0);    /* BTR0 for 500kbps */
   ts7800v2_write_reg(priv, SJA1000_BTR1, 0x1c); /* BTR1 for 500kbps */
   ts7800v2_write_reg(priv, SJA1000_EWL, 96);    /* Error warning limit */
   ts7800v2_write_reg(priv, SJA1000_RXERR, 0);   /* Reset RX error counter to 0 */
   ts7800v2_write_reg(priv, SJA1000_TXERR, 0);   /* Reset TX error counter to 0 */
   ts7800v2_write_reg(priv, SJA1000_ACCC0, 0);   /* Acceptance code */
   ts7800v2_write_reg(priv, SJA1000_ACCC1, 0);   /* Acceptance code */
   ts7800v2_write_reg(priv, SJA1000_ACCC2, 0);   /* Acceptance code */
   ts7800v2_write_reg(priv, SJA1000_ACCC3, 0);   /* Acceptance code */
   ts7800v2_write_reg(priv, SJA1000_ACCM0, 0);   /* Acceptance mask */
   ts7800v2_write_reg(priv, SJA1000_ACCM1, 0);   /* Acceptance mask */
   ts7800v2_write_reg(priv, SJA1000_ACCM2, 0);   /* Acceptance mask */
   ts7800v2_write_reg(priv, SJA1000_ACCM3, 0);   /* Acceptance mask */
   ts7800v2_write_reg(priv, SJA1000_MOD, 0);     /* Leave reset mode */

   platform_set_drvdata(pdev, ndev);
   SET_NETDEV_DEV(ndev, &pdev->dev);

   ret = register_sja1000dev(ndev);
   if (ret) {
      dev_err(&pdev->dev, "fail to register failed (err=%d)\n", ret);
      free_sja1000dev(ndev);
      return ret;
   }

   netdev_dbg(ndev, "reg_base=0x%p irq=%d\n",
         priv->reg_base, ndev->irq);

   return 0;
}


/**
 * ts7800v2_can_remove - Unregister the device after releasing the resources
 * @pdev:	Handle to the platform device structure
 *
 * This function frees all the resources allocated to the device.
 * Return: 0 always
 */
static int ts7800v2_can_remove(struct platform_device *pdev)
{
   struct net_device *dev = platform_get_drvdata(pdev);

#if (DEBUG_ON)
   printk("%s %d\n", __func__, __LINE__);
#endif

   unregister_sja1000dev(dev);
   free_sja1000dev(dev);

   return 0;
}

/* Match table for OF platform binding */
static const struct of_device_id ts7800v2_can_of_match[] = {
   { .compatible = "technologic,ts7800v2-can", },
   { /* end of list */ },
};
MODULE_DEVICE_TABLE(of, ts7800v2_can_of_match);

static struct platform_driver ts7800v2_can_driver = {
   .probe = ts7800v2_can_probe,
   .remove	= ts7800v2_can_remove,
   .driver	= {
      .name = DRIVER_NAME,
      .of_match_table	= ts7800v2_can_of_match,
   },
};

module_platform_driver(ts7800v2_can_driver);

MODULE_DESCRIPTION("TS7800v2 CAN interface");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Technologic Systems, Inc");

