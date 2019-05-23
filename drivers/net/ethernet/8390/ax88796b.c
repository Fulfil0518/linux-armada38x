/* drivers/net/ethernet/8390/ax88796.c
 *
 * Copyright 2005,2007 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * Asix AX88796 10/100 Ethernet controller support
 *	Based on ne.c, by Donald Becker, et-al.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/isapnp.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mdio-bitbang.h>
#include <linux/phy.h>
#include <linux/eeprom_93cx6.h>
#include <linux/slab.h>

#include <net/ax88796.h>



/* Rename the lib8390.c functions to show that they are in this driver */
#define __ei_open ax_ei_open
#define __ei_close ax_ei_close
#define __ei_poll ax_ei_poll
#define __ei_start_xmit ax_ei_start_xmit
#define __ei_tx_timeout ax_ei_tx_timeout
#define __ei_get_stats ax_ei_get_stats
#define __ei_set_multicast_list ax_ei_set_multicast_list
#define __ei_interrupt ax_ei_interrupt
#define ____alloc_ei_netdev ax__alloc_ei_netdev
#define __NS8390_init ax_NS8390_init



/* force unsigned long back to 'void __iomem *' */
#define ax_convert_addr(_a) ((void __force __iomem *)(_a))

#define ei_inb(_a) readb(ax_convert_addr(_a))
#define ei_outb(_v, _a) writeb(_v, ax_convert_addr(_a))

#define ei_inb_p(_a) ei_inb(_a)
#define ei_outb_p(_v, _a) ei_outb(_v, _a)

/* define EI_SHIFT() to take into account our register offsets */
#define EI_SHIFT(x) (ei_local->reg_offset[(x)])

#define E8390_PAGE3	0xc0	
#define EN0_DSR   EI_SHIFT(0X17) /* Read */	
#define EN0_BJLC  EI_SHIFT(0X17) /* Write */
#define EN0_MCR   EI_SHIFT(0X1b)  /* Mac configure register */
#define EN3_PMR   EI_SHIFT(0X0b) 
#define EN3_TBR   EI_SHIFT(0x0d)	/* Transmit Buffer Ring Control Register */

#define ENTBR_ENABLE    0x01			/* Enable Transmit Buffer Ring */
#define ENTQC_ENABLE    0x20		/* Enable TXQ */


#define NE_IO_EXTENT        0x20



/* Ensure we have our RCR base value */
#define AX88796_PLATFORM

static unsigned char version[] = "ax88796b.c: Copyright 2005,2007 Simtec Electronics\n";

#include "lib8390.c"

#define DRV_NAME "ax88796b"
#define DRV_VERSION "1.00"
#define MAX_AX_DEVICES 2

/* from ne.c */
#define NE_CMD		EI_SHIFT(0x00)
#define NE_RESET	EI_SHIFT(0x1f)
#define HWUR     EI_SHIFT(0x1f)
#define NE_DATAPORT	EI_SHIFT(0x10)

#define NE1SM_START_PG	0x20	/* First page of TX buffer */
#define NE1SM_STOP_PG	0x40	/* Last page +1 of RX ring */
#define NESM_START_PG	0x40	/* First page of TX buffer */
#define NESM_STOP_PG	0x80	/* Last page +1 of RX ring */

#define NESM_RX_START_PG	(NESM_START_PG + TX_PAGES)	/* First page of RX buffer */


#define AX_GPOC_PPDSET	BIT(6)

static u32 ax_msg_enable;

/* device private data */

struct ax_device {
	struct mii_bus *mii_bus;
	struct mdiobb_ctrl bb_ctrl;
	struct phy_device *phy_dev;
	void __iomem *addr_memr;
	u8 reg_memr;
	int link;
	int speed;
	int duplex;

	void __iomem *map2;
	const struct ax_plat_data *plat;

	unsigned char running;
	unsigned char resume_open;
	unsigned int irqflags;

	u32 reg_offsets[0x20];
};

static struct ax_device *ax_devices[MAX_AX_DEVICES];

static int pld_offsets[4] = { 0x280, 0x290, 0x29a, 0x29b };

static const struct of_device_id of_ax88796b_match[];

static struct ax_plat_data asix_platdata = {
	.flags		= 0, 
	.wordlength	= 1,
	.dcr_val = 0x48,
};

static inline struct ax_device *to_ax_dev(struct net_device *dev)
{
	struct ei_device *ei_local = netdev_priv(dev);
	return (struct ax_device *)(ei_local + 1);
}

/*
 * ax_initial_check
 *
 * do an initial probe for the card to check whether it exists
 * and is functional
 */
static int ax_initial_check(struct net_device *dev)
{
	struct ei_device *ei_local = netdev_priv(dev);
	//void __iomem *ioaddr = ei_local->mem;
	void __iomem *ioaddr = (void __iomem *)dev->base_addr;
	int reg0;
	int regd;

printk("%s %d\n", __func__, __LINE__);

	reg0 = ei_inb(ioaddr);
	
printk("%s %d, 0x%08X = 0x%08X\n", __func__, __LINE__, (unsigned int)ioaddr, (unsigned int)reg0);
	
	if (reg0 == 0xFF)
		return -ENODEV;
		
ei_outb(E8390_NODMA + E8390_PAGE3 + E8390_STOP, ioaddr + E8390_CMD);
regd = ei_inb(ioaddr + EN3_PMR);
printk("Read the PMR: 0x%02X\n", regd & 0xFF);   
		
	ei_outb(E8390_NODMA + E8390_PAGE1 + E8390_STOP, ioaddr + E8390_CMD);
	regd = ei_inb(ioaddr + EN0_COUNTER0);
	ei_outb(0xff, ioaddr + EN0_COUNTER0);
	ei_outb(E8390_NODMA + E8390_PAGE0, ioaddr + E8390_CMD);
	ei_inb(ioaddr + EN0_COUNTER0); /* Clear the counter by reading. */
	if (ei_inb(ioaddr + EN0_COUNTER0) != 0) {
		ei_outb(reg0, ioaddr);
		ei_outb(regd, ioaddr + 0x0d);	/* Restore the old values. */
		return -ENODEV;
	}

	ei_outb(E8390_NODMA + E8390_PAGE0 + E8390_STOP, ioaddr + E8390_CMD);
	printk("%s %d, ret = 0\n", __func__, __LINE__);
	return 0;
}

/*
 * Hard reset the card. This used to pause for the same period that a
 * 8390 reset command required, but that shouldn't be necessary.
 */

static void ax_reset_8390(struct net_device *dev)
{  
	struct ei_device *ei_local = netdev_priv(dev);
	unsigned long reset_start_time;
	void __iomem *addr = (void __iomem *)dev->base_addr;
	int z;
      
printk("%s %d, reset reg at 0x%08X\n", __func__, __LINE__, (unsigned int)(addr + NE_RESET));

	netif_dbg(ei_local, hw, dev, "resetting the 8390 t=%ld...\n", jiffies);

	/* read the NE_RESET to reset the chip; write to perform wakeup */
	
	//ei_outb(ei_inb(addr + NE_RESET), addr + NE_RESET);
   z = ei_inb(addr + NE_RESET);
   
   
	ei_local->txing = 0;
	ei_local->dmaing = 0;
	
	/* This check _should_not_ be necessary, omit eventually. */
	ei_outb(1, addr + HWUR);   /* write 1 to wake up the chip */
	
	reset_start_time = jiffies;
	while ((ei_inb(addr + EN0_ISR) & ENISR_RESET) == 0) { // wait for reset complete
		if (jiffies - reset_start_time > 2 * HZ / 100) {
			netdev_warn(dev, "%s: did not complete.\n", __func__);
			break;
		}
	}

	
	ei_outb(0xff, addr + EN0_ISR);	/* Ack all intr. */

   

	//ei_outb(z, addr + NE_RESET);
	
	ei_outb(ei_inb(addr + EN0_MCR) & ~(1<<4), addr + EN0_MCR);   /* select internal PHY.  This *should* be the default */
	
	printk("EN0_MCR: 0x%08X\n", (unsigned int)ei_inb(addr + EN0_MCR));
	
}


static void ax_get_8390_hdr(struct net_device *dev, struct e8390_pkt_hdr *hdr,
			    int ring_page)
{
	struct ei_device *ei_local = netdev_priv(dev);
	//void __iomem *nic_base = ei_local->mem + 0100;
	void __iomem *nic_base = (void __iomem *)dev->base_addr; 

	/* This *shouldn't* happen. If it does, it's the last thing you'll see */
	if (ei_local->dmaing) {
		netdev_err(dev, "DMAing conflict in %s "
			"[DMAstat:%d][irqlock:%d].\n",
			__func__,
			ei_local->dmaing, ei_local->irqlock);
		return;
	}

	ei_local->dmaing |= 0x01;
	ei_outb(E8390_NODMA + E8390_PAGE0 + E8390_START, nic_base + NE_CMD);
	ei_outb(sizeof(struct e8390_pkt_hdr), nic_base + EN0_RCNTLO);
	ei_outb(0, nic_base + EN0_RCNTHI);
	ei_outb(0, nic_base + EN0_RSARLO);		/* On page boundary */
	ei_outb(ring_page, nic_base + EN0_RSARHI);
	ei_outb(E8390_RREAD+E8390_START, nic_base + NE_CMD);
	
	if (ei_local->word16)
		ioread16_rep(nic_base + NE_DATAPORT, hdr,
			     sizeof(struct e8390_pkt_hdr) >> 1);
	else {
	//ei_outb(0x48, nic_base + EN0_DCFG);
	printk("calling ioread8_rep 0x%08X\n", (unsigned int)(nic_base + NE_DATAPORT));  
		ioread8_rep(nic_base + NE_DATAPORT, hdr,
			    sizeof(struct e8390_pkt_hdr));
		}

	ei_outb(ENISR_RDC, nic_base + EN0_ISR);	/* Ack intr. */
	ei_local->dmaing &= ~0x01;

	//{
	 //  int i;
	  // for(i=0; i < sizeof(struct e8390_pkt_hdr); i++)
	   //   printk("%02X ", ((unsigned char *)hdr)[i]);
//	}
//	printk("\n");
	
	le16_to_cpus(&hdr->count);
}


/*
 * Block input and output, similar to the Crynwr packet driver. If
 * you are porting to a new ethercard, look at the packet driver
 * source for hints. The NEx000 doesn't share the on-board packet
 * memory -- you have to put the packet out through the "remote DMA"
 * dataport using ei_outb.
 */
static void ax_block_input(struct net_device *dev, int count,
			   struct sk_buff *skb, int ring_offset)
{
	struct ei_device *ei_local = netdev_priv(dev);
	//void __iomem *nic_base = ei_local->mem;
	void __iomem *nic_base = (void __iomem *)dev->base_addr;
	char *buf = skb->data;

printk("%s %d, count: %d\n", __func__, __LINE__, count);	
	
	if (ei_local->dmaing) {
		netdev_err(dev,
			"DMAing conflict in %s "
			"[DMAstat:%d][irqlock:%d].\n",
			__func__,
			ei_local->dmaing, ei_local->irqlock);
		return;
	}

	ei_local->dmaing |= 0x01;

	ei_outb(E8390_NODMA+E8390_PAGE0+E8390_START, nic_base + NE_CMD);
	ei_outb(count & 0xff, nic_base + EN0_RCNTLO);
	ei_outb(count >> 8, nic_base + EN0_RCNTHI);
	ei_outb(ring_offset & 0xff, nic_base + EN0_RSARLO);
	ei_outb(ring_offset >> 8, nic_base + EN0_RSARHI);
	ei_outb(E8390_RREAD+E8390_START, nic_base + NE_CMD);

while (( ei_inb((unsigned long)nic_base+EN0_DSR) & 0x20) ==0);	

printk("%s %d\n", __func__, __LINE__);

	if (ei_local->word16) {
		ioread16_rep(nic_base + NE_DATAPORT, buf, count >> 1);
		if (count & 0x01)
			buf[count-1] = ei_inb(nic_base + NE_DATAPORT);

	} else {
		ioread8_rep(nic_base + NE_DATAPORT, buf, count);
	}

	ei_local->dmaing &= ~1;
}

static void ax_block_output(struct net_device *dev, int count,
			    const unsigned char *buf, const int start_page)
{
	struct ei_device *ei_local = netdev_priv(dev);
	//void __iomem *nic_base = ei_local->mem;
	void __iomem *nic_base = (void __iomem *)dev->base_addr;
	unsigned long dma_start;
	
printk("%s %d, count: %d, start_page: %d, nic_base: 0x%08X\n", __func__, __LINE__, count, start_page, (unsigned int)nic_base);
	/*
	 * Round the count up for word writes. Do we need to do this?
	 * What effect will an odd byte count have on the 8390?  I
	 * should check someday.
	 */
	if (ei_local->word16 && (count & 0x01))
		count++;

	/* This *shouldn't* happen. If it does, it's the last thing you'll see */
	if (ei_local->dmaing) {
		netdev_err(dev, "DMAing conflict in %s."
			"[DMAstat:%d][irqlock:%d]\n",
			__func__,
		       ei_local->dmaing, ei_local->irqlock);
		return;
	}

	ei_local->dmaing |= 0x01;
	/* We should already be in page 0, but to be safe... */
	ei_outb(E8390_PAGE0+E8390_START+E8390_NODMA, nic_base + NE_CMD);

	ei_outb(ENISR_RDC, nic_base + EN0_ISR); /* Remote DMA Complete */

	/* Now the normal output. */
	ei_outb(count & 0xff, nic_base + EN0_RCNTLO); /* Remote Byte Count Reg 0 */
	ei_outb(count >> 8, nic_base + EN0_RCNTHI);   /* Remote Byte Count Reg 1 */
	ei_outb(0x00, nic_base + EN0_RSARLO);      /* Remote Start Address Reg 0 */
	ei_outb(start_page, nic_base + EN0_RSARHI);/* Remote Start Address Reg 1 */

	ei_outb(E8390_RWRITE+E8390_START, nic_base + NE_CMD);  /* Remote write  */
	if (ei_local->word16)
		iowrite16_rep(nic_base + NE_DATAPORT, buf, count >> 1);
	else
		iowrite8_rep(nic_base + NE_DATAPORT, buf, count);

	dma_start = jiffies;

	while ((ei_inb(nic_base + EN0_ISR) & ENISR_RDC) == 0) {
		if (jiffies - dma_start > 2 * HZ / 100) {		/* 20ms */
			netdev_warn(dev, "timeout waiting for Tx RDC.\n");
			ax_reset_8390(dev);
			ax_NS8390_init(dev, 1);
			break;
		}
	}

	ei_outb(ENISR_RDC, nic_base + EN0_ISR);	/* Ack intr. */
	ei_local->dmaing &= ~0x01;
}

/* definitions for accessing MII/EEPROM interface */

#define AX_MEMR			EI_SHIFT(0x14)
#define AX_MEMR_MDC		BIT(0)
#define AX_MEMR_MDIR		BIT(1)
#define AX_MEMR_MDI		BIT(2)
#define AX_MEMR_MDO		BIT(3)
#define AX_MEMR_EECS		BIT(4)
#define AX_MEMR_EEI		BIT(5)
#define AX_MEMR_EEO		BIT(6)
#define AX_MEMR_EECLK		BIT(7)

static void ax_handle_link_change(struct net_device *dev)
{
	struct ax_device  *ax = to_ax_dev(dev);
	struct phy_device *phy_dev = ax->phy_dev;
	int status_change = 0;

	if (phy_dev->link && ((ax->speed != phy_dev->speed) ||
			     (ax->duplex != phy_dev->duplex))) {

		ax->speed = phy_dev->speed;
		ax->duplex = phy_dev->duplex;
		status_change = 1;
	}

	if (phy_dev->link != ax->link) {
		if (!phy_dev->link) {
			ax->speed = 0;
			ax->duplex = -1;
		}
		ax->link = phy_dev->link;

		status_change = 1;
	}

	if (status_change)
		phy_print_status(phy_dev);
}

static int ax_mii_probe(struct net_device *dev)
{
   struct ei_device *ei_local = netdev_priv(dev);
	struct ax_device  *ax = to_ax_dev(dev);
	struct phy_device *phy_dev = NULL;
	int ret, regd;
   void __iomem *addr = (void __iomem *)dev->base_addr;
   
	/* find the first phy */
	phy_dev = phy_find_first(ax->mii_bus);
	if (!phy_dev) {
		netdev_err(dev, "no PHY found\n");
		return -ENODEV;
	}

   printk("Found a PHY: %s\n", phydev_name(phy_dev));

#if (1)

   ei_outb(1, addr + NE_RESET);  // do wakeup
   
   printk("MII_PHYSID1: 0x%08X\n", (unsigned int)phy_read(phy_dev, MII_PHYSID1));
   printk("MII_PHYSID2: 0x%08X\n", (unsigned int)phy_read(phy_dev, MII_PHYSID2));

ei_outb(E8390_NODMA + E8390_PAGE3 + E8390_START, addr + E8390_CMD);
regd = ei_inb(addr + EN3_PMR);
printk("Read the PMR: 0x%02X\n", regd & 0xFF);   
	
/* Select page 0 */
ei_outb(E8390_NODMA + E8390_PAGE0 + E8390_START, addr + E8390_CMD);
#endif

printk("AX88796B:  Variables connecting to phy:  dev, phy_dev, ax_handle_link_change, PHY_INTERFACE_MODE_MII, 0x%x, 0x%x, 0x%x, 0x%x.\n", dev, phy_dev, ax_handle_link_change, PHY_INTERFACE_MODE_MII);

	ret = phy_connect_direct(dev, phy_dev, ax_handle_link_change,
				 PHY_INTERFACE_MODE_MII);
	if (ret) {
		netdev_err(dev, "Could not attach to PHY\n");
		return ret;
	}

	/* mask with MAC supported features */
	phy_dev->supported &= PHY_BASIC_FEATURES;
	phy_dev->advertising = phy_dev->supported;

	ax->phy_dev = phy_dev;
	netdev_err(dev, "PHY driver [%s] (mii_bus:phy_addr=%s, irq=%d)\n",
//	netdev_info(dev, "PHY driver [%s] (mii_bus:phy_addr=%s, irq=%d)\n",
		    phy_dev->drv->name, phydev_name(phy_dev), phy_dev->irq);

	return 0;
}

static void ax_phy_switch(struct net_device *dev, int on)
{
   void __iomem *ioaddr = (void __iomem *)dev->base_addr;

	struct ei_device *ei_local = netdev_priv(dev);
	struct ax_device *ax = to_ax_dev(dev);
   int regd;
   
	u8 reg_gpoc =  ax->plat->gpoc_val;

	if (!!on)
		reg_gpoc &= ~AX_GPOC_PPDSET;
	else
		reg_gpoc |= AX_GPOC_PPDSET;

printk("%s %d %s\n", __func__, __LINE__, on?"ON":"OFF");		

ei_outb(E8390_NODMA + E8390_PAGE3 + E8390_STOP, ioaddr + E8390_CMD);
regd = ei_inb(ioaddr + EN3_PMR);


if (on)    
   regd &= ~3;
   
printk("Read the PMR at 0x%08X: 0x%02X\n", (unsigned int)ioaddr + EN3_PMR, regd & 0xFF);   

ei_outb(regd, ioaddr + EN3_PMR);

	//ei_outb(reg_gpoc, ei_local->mem + EN0_BJLC);
}

static int ax_open(struct net_device *dev)
{
	struct ax_device *ax = to_ax_dev(dev);
	int ret;

	netdev_dbg(dev, "open\n");

	ret = request_irq(dev->irq, ax_ei_interrupt, ax->irqflags,
			  dev->name, dev);
	if (ret)
		goto failed_request_irq;

	/* turn the phy on (if turned off) */
	ax_phy_switch(dev, 1);

	ret = ax_mii_probe(dev);
	if (ret)
		goto failed_mii_probe;
	phy_start(ax->phy_dev);

	ret = ax_ei_open(dev);
	if (ret)
		goto failed_ax_ei_open;

	ax->running = 1;

	return 0;

 failed_ax_ei_open:
	phy_disconnect(ax->phy_dev);
 failed_mii_probe:
	ax_phy_switch(dev, 0);
	free_irq(dev->irq, dev);
 failed_request_irq:
	return ret;
}

static int ax_close(struct net_device *dev)
{
	struct ax_device *ax = to_ax_dev(dev);

	netdev_dbg(dev, "close\n");

	ax->running = 0;
	wmb();

	ax_ei_close(dev);

	/* turn the phy off */
	ax_phy_switch(dev, 0);
	phy_disconnect(ax->phy_dev);

	free_irq(dev->irq, dev);
	return 0;
}

static int ax_ioctl(struct net_device *dev, struct ifreq *req, int cmd)
{
	struct ax_device *ax = to_ax_dev(dev);
	struct phy_device *phy_dev = ax->phy_dev;

	if (!netif_running(dev))
		return -EINVAL;

	if (!phy_dev)
		return -ENODEV;

	return phy_mii_ioctl(phy_dev, req, cmd);
}

/* ethtool ops */

static void ax_get_drvinfo(struct net_device *dev,
			   struct ethtool_drvinfo *info)
{
	struct platform_device *pdev = to_platform_device(dev->dev.parent);

	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
	strlcpy(info->bus_info, pdev->name, sizeof(info->bus_info));
}

static int ax_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct ax_device *ax = to_ax_dev(dev);
	struct phy_device *phy_dev = ax->phy_dev;

	if (!phy_dev)
		return -ENODEV;

	return phy_ethtool_gset(phy_dev, cmd);
}

static int ax_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct ax_device *ax = to_ax_dev(dev);
	struct phy_device *phy_dev = ax->phy_dev;

	if (!phy_dev)
		return -ENODEV;

	return phy_ethtool_sset(phy_dev, cmd);
}

static u32 ax_get_msglevel(struct net_device *dev)
{
	struct ei_device *ei_local = netdev_priv(dev);

	return ei_local->msg_enable;
}

static void ax_set_msglevel(struct net_device *dev, u32 v)
{
	struct ei_device *ei_local = netdev_priv(dev);

	ei_local->msg_enable = v;
}

static const struct ethtool_ops ax_ethtool_ops = {
	.get_drvinfo		= ax_get_drvinfo,
	.get_settings		= ax_get_settings,
	.set_settings		= ax_set_settings,
	.get_link		= ethtool_op_get_link,
	.get_ts_info		= ethtool_op_get_ts_info,
	.get_msglevel		= ax_get_msglevel,
	.set_msglevel		= ax_set_msglevel,
};

#ifdef CONFIG_AX88796_93CX6
static void ax_eeprom_register_read(struct eeprom_93cx6 *eeprom)
{
	struct ei_device *ei_local = eeprom->data;
	u8 reg = ei_inb(ei_local->mem + AX_MEMR);

	eeprom->reg_data_in = reg & AX_MEMR_EEI;
	eeprom->reg_data_out = reg & AX_MEMR_EEO; /* Input pin */
	eeprom->reg_data_clock = reg & AX_MEMR_EECLK;
	eeprom->reg_chip_select = reg & AX_MEMR_EECS;
}

static void ax_eeprom_register_write(struct eeprom_93cx6 *eeprom)
{
	struct ei_device *ei_local = eeprom->data;
	u8 reg = ei_inb(ei_local->mem + AX_MEMR);

	reg &= ~(AX_MEMR_EEI | AX_MEMR_EECLK | AX_MEMR_EECS);

	if (eeprom->reg_data_in)
		reg |= AX_MEMR_EEI;
	if (eeprom->reg_data_clock)
		reg |= AX_MEMR_EECLK;
	if (eeprom->reg_chip_select)
		reg |= AX_MEMR_EECS;

	ei_outb(reg, ei_local->mem + AX_MEMR);
	udelay(10);
}
#endif

static const struct net_device_ops ax_netdev_ops = {
	.ndo_open		= ax_open,
	.ndo_stop		= ax_close,
	.ndo_do_ioctl		= ax_ioctl,

	.ndo_start_xmit		= ax_ei_start_xmit,
	.ndo_tx_timeout		= ax_ei_tx_timeout,
	.ndo_get_stats		= ax_ei_get_stats,
	.ndo_set_rx_mode	= ax_ei_set_multicast_list,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_change_mtu		= eth_change_mtu,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= ax_ei_poll,
#endif
};

static void ax_bb_mdc(struct mdiobb_ctrl *ctrl, int level)
{
	struct ax_device *ax = container_of(ctrl, struct ax_device, bb_ctrl);

	if (level)
		ax->reg_memr |= AX_MEMR_MDC;
	else
		ax->reg_memr &= ~AX_MEMR_MDC;

	//printk("%s %d: ax->reg_memr=0x%08lX, ax->addr_memr=0x%08lX\n", __func__, __LINE__, ax->reg_memr, ax->addr_memr);

	ei_outb(ax->reg_memr, ax->addr_memr);
}

static void ax_bb_dir(struct mdiobb_ctrl *ctrl, int output)
{
	struct ax_device *ax = container_of(ctrl, struct ax_device, bb_ctrl);

	if (output)
		ax->reg_memr &= ~AX_MEMR_MDIR;
	else
		ax->reg_memr |= AX_MEMR_MDIR;

	ei_outb(ax->reg_memr, ax->addr_memr);
}

static void ax_bb_set_data(struct mdiobb_ctrl *ctrl, int value)
{
	struct ax_device *ax = container_of(ctrl, struct ax_device, bb_ctrl);

	if (value)
		ax->reg_memr |= AX_MEMR_MDO;
	else
		ax->reg_memr &= ~AX_MEMR_MDO;

	ei_outb(ax->reg_memr, ax->addr_memr);
}

static int ax_bb_get_data(struct mdiobb_ctrl *ctrl)
{
	struct ax_device *ax = container_of(ctrl, struct ax_device, bb_ctrl);
	int reg_memr = ei_inb(ax->addr_memr);
		
	return reg_memr & AX_MEMR_MDI ? 1 : 0;
}

static struct mdiobb_ops bb_ops = {
	.owner = THIS_MODULE,
	.set_mdc = ax_bb_mdc,
	.set_mdio_dir = ax_bb_dir,
	.set_mdio_data = ax_bb_set_data,
	.get_mdio_data = ax_bb_get_data,
};

/* setup code */

static int ax_mii_init(struct net_device *dev)
{
	struct platform_device *pdev = to_platform_device(dev->dev.parent);
	struct ei_device *ei_local = netdev_priv(dev);
	struct ax_device *ax = to_ax_dev(dev);
	int err, i;

	ax->bb_ctrl.ops = &bb_ops;
	//ax->addr_memr = ei_local->mem + AX_MEMR;
	ax->addr_memr = (void __iomem *)(dev->base_addr + AX_MEMR);
	ax->mii_bus = alloc_mdio_bitbang(&ax->bb_ctrl);
	if (!ax->mii_bus) {
		err = -ENOMEM;
		goto out;
	}

printk("%s 	ax->addr_memr = 0x%08X\n", __func__, (unsigned int)ax->addr_memr);

	ax->mii_bus->name = "ax88796_mii_bus";
	ax->mii_bus->parent = dev->dev.parent;
	snprintf(ax->mii_bus->id, MII_BUS_ID_SIZE, "%s-%x",
		pdev->name, pdev->id);

#if (0)

   // mii_bus->irq is now an array if int 
	ax->mii_bus->irq = kmalloc(sizeof(int) * PHY_MAX_ADDR, GFP_KERNEL);
	if (!ax->mii_bus->irq) {
		err = -ENOMEM;
		goto out_free_mdio_bitbang;
	}
#endif

	for (i = 0; i < PHY_MAX_ADDR; i++)
		ax->mii_bus->irq[i] = PHY_POLL;

	err = mdiobus_register(ax->mii_bus);
	if (err)
		goto out_free_irq;

	return 0;

 out_free_irq:
	kfree(ax->mii_bus->irq);
	free_mdio_bitbang(ax->mii_bus);
 out:
	return err;
}

static void ax_initial_setup(struct net_device *dev, struct ei_device *ei_local)
{
	//void __iomem *ioaddr = ei_local->mem;
	void __iomem *ioaddr = (void __iomem *)dev->base_addr;
	struct ax_device *ax = to_ax_dev(dev);

	/* Select page 0 */
	ei_outb(E8390_NODMA + E8390_PAGE0 + E8390_STOP, ioaddr + E8390_CMD);

	/* set to byte access */
//	ei_outb(ax->plat->dcr_val & ~1, ioaddr + EN0_DCFG);



ei_outb(ax->plat->dcr_val, ioaddr + EN0_DCFG);
printk("%s %d write 0x%08X to 0x%08X\n", __func__, __LINE__, (unsigned int)ax->plat->dcr_val, (unsigned int)ioaddr + EN0_DCFG);	
	//ei_outb(ax->plat->gpoc_val, ioaddr + EN0_BJLC);
	

}

/*
 * ax_init_dev
 *
 * initialise the specified device, taking care to note the MAC
 * address it may already have (if configured), ensure
 * the device is ready to be used by lib8390.c and registerd with
 * the network layer.
 */
static int ax_init_dev(struct net_device *dev)
{
	struct ei_device *ei_local = netdev_priv(dev);
	struct ax_device *ax = to_ax_dev(dev);
	//void __iomem *ioaddr = ei_local->mem;
	void __iomem *ioaddr = (void __iomem *)dev->base_addr;
	unsigned int start_page;
	unsigned int stop_page;
	int ret;
	int i;

	ret = ax_initial_check(dev);
	if (ret)
		goto err_out;
printk("%s %d\n", __func__, __LINE__);
	/* setup goes here */

	ax_initial_setup(dev, ei_local);
	
	/* read the mac from the card prom if we need it */

	if (ax->plat->flags & AXFLG_HAS_EEPROM) {
		unsigned char SA_prom[32];

		for (i = 0; i < sizeof(SA_prom); i += 2) {
			SA_prom[i] = ei_inb(ioaddr + NE_DATAPORT);
			SA_prom[i + 1] = ei_inb(ioaddr + NE_DATAPORT);
		}

		if (ax->plat->wordlength == 2)
			for (i = 0; i < 16; i++)
				SA_prom[i] = SA_prom[i+i];

		memcpy(dev->dev_addr, SA_prom, ETH_ALEN);
	}

#ifdef CONFIG_AX88796_93CX6
	if (ax->plat->flags & AXFLG_HAS_93CX6) {
		unsigned char mac_addr[ETH_ALEN];
		struct eeprom_93cx6 eeprom;

		eeprom.data = ei_local;
		eeprom.register_read = ax_eeprom_register_read;
		eeprom.register_write = ax_eeprom_register_write;
		eeprom.width = PCI_EEPROM_WIDTH_93C56;

		eeprom_93cx6_multiread(&eeprom, 0,
				       (__le16 __force *)mac_addr,
				       sizeof(mac_addr) >> 1);

		memcpy(dev->dev_addr, mac_addr, ETH_ALEN);
	}
#endif
printk("%s %d write 0x%08X to 0x%08X\n", __func__, __LINE__, (unsigned int)ax->plat->dcr_val, (unsigned int)ioaddr + EN0_DCFG);
   ei_outb(ax->plat->dcr_val, ioaddr + EN0_DCFG);

	//if (ax->plat->wordlength == 2) {
		/* We must set the 8390 for word mode. */
		//ei_outb(ax->plat->dcr_val, ei_local->mem + EN0_DCFG);
		//_outb(ax->plat->dcr_val, ioaddr + EN0_DCFG);
		start_page = NESM_START_PG;
		stop_page = NESM_STOP_PG;
	//} else {		
		//start_page = NE1SM_START_PG;
		//stop_page = NE1SM_STOP_PG;
	//}

	/* load the mac-address from the device */
	if (ax->plat->flags & AXFLG_MAC_FROMDEV) {
		//ei_outb(E8390_NODMA + E8390_PAGE1 + E8390_STOP, ei_local->mem + E8390_CMD); /* 0x61 */
		ei_outb(E8390_NODMA + E8390_PAGE1 + E8390_STOP, ioaddr + E8390_CMD); /* 0x61 */
		for (i = 0; i < ETH_ALEN; i++)
			dev->dev_addr[i] =
				ei_inb(ioaddr + EN1_PHYS_SHIFT(i));
	}

	if ((ax->plat->flags & AXFLG_MAC_FROMPLATFORM) &&
	    ax->plat->mac_addr)
		memcpy(dev->dev_addr, ax->plat->mac_addr, ETH_ALEN);
printk("%s %d\n", __func__, __LINE__);

	ax_reset_8390(dev);

ei_local->bigendian = 0; //(ax->plat->flags & AXFLG_BIGENDIAN)? 1:0;
printk("ei_local->bigendian: %s\n", ei_local->bigendian? "YES":"NO");

	ei_local->name = "AX88796b";
	ei_local->tx_start_page = start_page;
	ei_local->stop_page = stop_page;
	ei_local->word16 = (ax->plat->wordlength == 2);
	ei_local->rx_start_page = start_page + TX_PAGES;

//	printk("tx_start_page = %d, rx_start_page = %d, stop_page = %d, TX_PAGES = %d\n", ei_local->tx_start_page , ei_local->rx_start_page, ei_local->stop_page, TX_PAGES);
	
	
#ifdef PACKETBUF_MEMSIZE
	/* Allow the packet buffer size to be overridden by know-it-alls. */
	ei_local->stop_page = ei_local->tx_start_page + PACKETBUF_MEMSIZE;
#endif

	ei_local->reset_8390 = &ax_reset_8390;
	ei_local->block_input = &ax_block_input;
	ei_local->block_output = &ax_block_output;
	ei_local->get_8390_hdr = &ax_get_8390_hdr;
	ei_local->priv = 0;
	ei_local->msg_enable = ax_msg_enable;

	dev->netdev_ops = &ax_netdev_ops;
	dev->ethtool_ops = &ax_ethtool_ops;
	
	ret = ax_mii_init(dev);
	if (ret)
		goto out_irq;
printk("%s %d\n", __func__, __LINE__);

	ax_NS8390_init(dev, 0);

	ret = register_netdev(dev);
	if (ret)
		goto out_irq;
printk("%s %d\n", __func__, __LINE__);

	netdev_info(dev, "%dbit, irq %d, %lx, MAC: %pM\n",
		    ei_local->word16 ? 16 : 8, dev->irq, dev->base_addr,
		    dev->dev_addr);
printk("%s %d return 0\n", __func__, __LINE__);


/* Select page 0 */
ei_outb(E8390_NODMA + E8390_PAGE0 + E8390_START, ioaddr + E8390_CMD);

	return 0;

 out_irq:
	/* cleanup irq */
	free_irq(dev->irq, dev);
 err_out:
	return ret;
}

static int ax_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct ei_device *ei_local = netdev_priv(dev);
	struct ax_device *ax = to_ax_dev(dev);
	struct resource *mem;

	unregister_netdev(dev);
	free_irq(dev->irq, dev);

	iounmap(ei_local->mem);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, resource_size(mem));

	if (ax->map2) {
		iounmap(ax->map2);
		mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		release_mem_region(mem->start, resource_size(mem));
	}

	free_netdev(dev);

	return 0;
}

/*
 * ax_probe
 *
 * This is the entry point when the platform device system uses to
 * notify us of a new device to attach to. Allocate memory, find the
 * resources and information passed, and map the necessary registers.
 */
static int ax_probe(struct platform_device *pdev)
{
   const struct of_device_id *match;
	struct net_device *dev;
	struct ei_device *ei_local;
	struct ax_device *ax;
	struct resource *mem, *mem2, *irq;
	unsigned long mem_size, mem2_size = 0;
	int ret = 0;
	int i;
	
	printk("%s %d\n", __func__, __LINE__);
	
   match = of_match_device(of_ax88796b_match, &pdev->dev);
   if (match == NULL) {
      dev_err(&pdev->dev, "No match in device-tree\n");
      return -ENODEV;
   }
   
   dev = ax__alloc_ei_netdev(sizeof(struct ax_device));
	if (dev == NULL)
		return -ENOMEM;
   
printk("%s &asix_platdata is 0x%08X\n", __func__, (unsigned int)&asix_platdata);

	/* ok, let's setup our device */
	SET_NETDEV_DEV(dev, &pdev->dev);
	ei_local = netdev_priv(dev);
	ax = to_ax_dev(dev);	
	ax->plat = (const struct ax_plat_data*)((struct platform_device_id*)(match->data))->driver_data;
		
   if (ax->plat == NULL) {                  
     return -ENOMEM;      
   }

printk("ax->plat = 0x%08X\n", (unsigned int)ax->plat);   
   
	ei_local->rxcr_base = ax->plat->rcr_val;
				
	
	
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
printk("mem = 0x%08X\n", (unsigned int)mem);	
	
	if (!mem) {
		dev_err(&pdev->dev, "no MEM specified\n");
		ret = -ENXIO;
		goto exit_mem;
	}

	mem_size = resource_size(mem);

printk("mem->start = 0x%08X\n", (unsigned int)mem->start);
printk("mem_size = 0x%08X\n", (unsigned int)mem_size);
  
	/*
	 * setup the register offsets from either the platform data or
	 * by using the size of the resource provided
	 */
	if (ax->plat->reg_offsets)
		ei_local->reg_offset = ax->plat->reg_offsets;
	else {
		ei_local->reg_offset = ax->reg_offsets;
		for (ret = 0; ret < 0x20; ret++) {
			ax->reg_offsets[ret] = (NE_IO_EXTENT / 0x20) * ret;
//printk("A: reg_offset[0x%02X] = 0x%08X\n", ret, (unsigned int)ax->reg_offsets[ret]); 			
			}
	}

	if (!request_mem_region(mem->start, mem_size, pdev->name)) {
		dev_err(&pdev->dev, "cannot reserve registers\n");
		ret = -ENXIO;
		goto exit_mem;
	}

	ei_local->mem = ioremap_nocache(mem->start, mem_size);
	if (ei_local->mem == NULL) {
		dev_err(&pdev->dev, "Cannot ioremap_nocache area %pR\n", mem);

		ret = -ENXIO;
		goto exit_req;
	}
	   
	dev->base_addr = (unsigned long)ei_local->mem;

printk("dev->base_addr = 0x%08X\n", (unsigned int)dev->base_addr);
	
	/* look for reset area */
	mem2 = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	
printk("mem2 = 0x%08X\n", (unsigned int)mem2);

	if (!mem2) {
		if (!ax->plat->reg_offsets) {
			for (ret = 0; ret < 0x20; ret++) {
				ax->reg_offsets[ret] = (NE_IO_EXTENT / 0x20) * ret;
			//printk("B: reg_offset[0x%02X] = 0x%08X\n", ret, (unsigned int)ax->reg_offsets[ret]); 			

				}
		}
	} else {
		mem2_size = resource_size(mem2);

		if (!request_mem_region(mem2->start, mem2_size, pdev->name)) {
			dev_err(&pdev->dev, "cannot reserve registers\n");
			ret = -ENXIO;
			goto exit_mem1;
		}

		ax->map2 = ioremap_nocache(mem2->start, mem2_size);
		if (!ax->map2) {
			dev_err(&pdev->dev, "cannot map reset register\n");
			ret = -ENXIO;
			goto exit_mem2;
		}

		ei_local->reg_offset[0x1f] = ax->map2 - ei_local->mem;
	}
printk("%s %d\n", __func__, __LINE__);
	/* got resources, now initialise and register device */
	
	dev->irq = 122;
			
	for(i=0; i < 4; i++) {
	   int pld = pld_offsets[i];
	   unsigned char r = ei_inb(ei_local->mem + pld);
	   
	   if ((r & 0xf0) == 0x20) {
	      unsigned char jp =  ei_inb(ei_local->mem + pld + 2);;
	      printk("Found TS-ETH2 at 0x%04X: 0x%02X\n", pld, r);

	      printk("\tJumpers: %s%s%s%s%s%s\n", 
	         (jp & BIT(0))?"JP1,":"",
  	         (jp & BIT(1))?"JP2,":"",
	         (jp & BIT(2))?"ARM,":"",
	         (jp & BIT(3))?"IRQ5,":"",
	         (jp & BIT(4))?"IRQ6,":"",
	         (jp & BIT(5))?"IRQ7":"");

	      switch(jp & 3) {
	          case 0:   dev->base_addr = (unsigned long)ei_local->mem + 0x100; break;
	          case 1:   dev->base_addr = (unsigned long)ei_local->mem + 0x180; break;
	          case 2:   dev->base_addr = (unsigned long)ei_local->mem + 0x200; break;
	          case 3:   dev->base_addr = (unsigned long)ei_local->mem + 0x300; break;
	      }
	      
	      if (jp & BIT(2)) {
	         switch((jp >> 3) & 0x7) {
	            case 0:
	            case 2:   dev->irq = 122; break;
	            case 1:
	            case 4:
	            case 5:   dev->irq = 123; break;
	            default:  dev->irq = 124;	            
	         }
	      } else {
	         switch((jp >> 3) & 0x3) {
	         case 0:  
	         case 2:   dev->irq = 122; break;
	         case 1:
	         case 3:   dev->irq = 124; break;
	         }
	      }
	      printk("Using offset 0x%03X and IRQ #%d\n", 
	         (unsigned int)dev->base_addr & 0xFFF, dev->irq);
	   }	   
	}
	
	ax->irqflags = IRQF_SHARED;
	
	platform_set_drvdata(pdev, dev);
				
	ret = ax_init_dev(dev);
	if (!ret)
		return 0;
printk("%s %d, ret = %d\n", __func__, __LINE__, ret);
	if (!ax->map2) {
printk("%s %d\n", __func__, __LINE__);	
		goto exit_mem1;
		}
printk("%s %d\n", __func__, __LINE__);
	iounmap(ax->map2);

 exit_mem2:
	release_mem_region(mem2->start, mem2_size);

 exit_mem1:
	iounmap(ei_local->mem);

 exit_req:
	release_mem_region(mem->start, mem_size);

 exit_mem:
	free_netdev(dev);

printk("%s %d ret = %d\n", __func__, __LINE__, ret);	
	return ret;
}

/* suspend and resume */

#ifdef CONFIG_PM
static int ax_suspend(struct platform_device *dev, pm_message_t state)
{
	struct net_device *ndev = platform_get_drvdata(dev);
	struct ax_device *ax = to_ax_dev(ndev);

	ax->resume_open = ax->running;

	netif_device_detach(ndev);
	ax_close(ndev);

	return 0;
}

static int ax_resume(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct ax_device *ax = to_ax_dev(ndev);

	ax_initial_setup(ndev, netdev_priv(ndev));
	ax_NS8390_init(ndev, ax->resume_open);
	netif_device_attach(ndev);

	if (ax->resume_open)
		ax_open(ndev);

	return 0;
}

#else
#define ax_suspend NULL
#define ax_resume NULL
#endif



static const struct platform_device_id ax88796b_devtype[] = {
   {
      .name = "ax88796b",
      .driver_data = (kernel_ulong_t)&asix_platdata,
   }, {
      /* sentinel */
   }
};
MODULE_DEVICE_TABLE(platform, ax88796b_devtype);

static const struct of_device_id of_ax88796b_match[] = {
   {
      .compatible = "asix,ax88796b",
      .data = &ax88796b_devtype[0],
   }, {
   },
};
MODULE_DEVICE_TABLE(of, of_ax88796b_match);

static struct platform_driver axdrv = {
	.driver	= {
		.name		= "ax88796b",
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(of_ax88796b_match),
	},
	.probe		= ax_probe,
	.remove		= ax_remove,
	.suspend	= ax_suspend,
	.resume		= ax_resume,
};

module_platform_driver(axdrv);

MODULE_DESCRIPTION("AX88796 10/100 Ethernet platform driver");
MODULE_AUTHOR("Ben Dooks, <ben@simtec.co.uk>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ax88796");
