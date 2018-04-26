/*
 * Copyright (C) 2017 Technologic Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/serial.h>
#include <linux/tty.h>
#include <linux/io.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <asm/bitops.h>
#include <asm/io.h>

//#define DEBUG

#define DRIVER_NAME "ts7800_isa16550"
#define NR_UARTS  4

static const char banner[] __initconst = "TS-7800-V2 ISA 16550 Uart Driver";

struct isa16550_uart_device {
   struct uart_8250_port port;
   int num;
};

static struct isa16550_uart_device isa16550_uart_devices[NR_UARTS];

/**
   The defaults below assume that the TS-SER4 has COM1 base at 0x3f8 (and
   so the remaining three uarts are at 0x2f8, 0x3e7, and 0x2e8).  This
   may be overridden with the module parameters (see the end of this file).

   The defaults further assume that all COM interrupts are routed to
   IRQ #5.  This, too, may be overridden with module parameters.
*/

static unsigned int io[NR_UARTS]  = { 0x3f8, 0x2f8, 0x3e8, 0x2e8 };
static unsigned int irq[NR_UARTS] = { 5, 5, 5, 5 };

static int io_cnt = 0;

void __iomem  *membase;

static int isa16550_probe(struct platform_device *dev);
static int isa16550_remove(struct platform_device *dev);

static struct uart_driver isa16550_uart_driver;

static const struct of_device_id of_ts7800_isa16550_match[] = {
   {
      .compatible = "technologicsystems,isa16550",
   },
};

static struct platform_driver isa16550_platform_driver = {
   .probe		= isa16550_probe,
   .remove		= isa16550_remove,
   .driver		= {
      .name	= "ts7800_isa16550",
      .of_match_table = of_match_ptr(of_ts7800_isa16550_match),
   },
};


static int __init ts7800_isa16550_init(void)
{
   int ret;

   pr_info("%s\n", banner);

   ret = uart_register_driver(&isa16550_uart_driver);
   if (likely(ret == 0)) {
      ret = platform_driver_register(&isa16550_platform_driver);
      if (unlikely(ret))
         uart_unregister_driver(&isa16550_uart_driver);
   }

   return ret;
}

static void __exit ts7800_isa16550_exit(void)
{
#ifdef DEBUG
   printk("%s %d\n", __func__, __LINE__);
#endif

   platform_driver_unregister(&isa16550_platform_driver);
   uart_unregister_driver(&isa16550_uart_driver);
}


static int isa16550_probe(struct platform_device *pdev)
{
   int n, nr_uarts;
   struct resource *res;
   struct device_node *np;
   const struct of_device_id *of_id;

#ifdef DEBUG
   printk("%s %d\n", __func__, __LINE__);
#endif
   of_id = of_match_device(of_ts7800_isa16550_match, &pdev->dev);

   if (! of_id)
      return -EINVAL;

   if ((res=platform_get_resource(pdev, IORESOURCE_MEM, 0)) == NULL) {
      pr_err("Can't get IORESOURCE_MEM\n");
      return -EINVAL;
   }

   np = pdev->dev.of_node;

   if (of_property_read_u32(np, "isa16550,nports", &nr_uarts) < 0) {
      pr_info("Can't read property 'isa16550,nports' in device-tree; assuming %d\n", NR_UARTS);
      nr_uarts = NR_UARTS;
   } else if (nr_uarts > NR_UARTS) {
      pr_warn("Property 'isa16550,nports' in device-tree too great; setting default of %d\n", NR_UARTS);
      nr_uarts = NR_UARTS;
   }

   if (io_cnt > 0) {
      pr_info("Setting-up %d uarts based on command-line\n", io_cnt);

      if (io_cnt >  NR_UARTS) {
         pr_warn("Too many addresses specified on command-line; setting default of %d\n", NR_UARTS);
         nr_uarts = NR_UARTS;
      } else {
         nr_uarts = io_cnt;
      }
   }

   membase = devm_ioremap(&pdev->dev, res->start, resource_size(res));

   if (IS_ERR(membase)) {
      pr_err("Could not map resource\n");
      return -ENOMEM;;
   }
#ifdef DEBUG
   printk("mapped %d bytes at 0x%08X to 0x%08X\n",  (int)resource_size(res), (unsigned int)res->start, (unsigned int)membase);
#endif
   memset(&isa16550_uart_devices, 0, sizeof(isa16550_uart_devices));

   for(n=0; n < nr_uarts; n++) {
      isa16550_uart_devices[n].port.port.dev = &pdev->dev;
      isa16550_uart_devices[n].port.port.type = PORT_16550A;
      isa16550_uart_devices[n].port.port.iotype = UPIO_MEM;
      isa16550_uart_devices[n].port.port.iobase = 0;

      isa16550_uart_devices[n].port.port.fifosize = 16;
      isa16550_uart_devices[n].port.port.flags =  UPF_FIXED_TYPE | UPF_SHARE_IRQ;
      isa16550_uart_devices[n].port.port.regshift = 0;
      isa16550_uart_devices[n].port.port.irq = 117 + irq[n];

      isa16550_uart_devices[n].port.port.mapbase = res->start + io[n];
      isa16550_uart_devices[n].port.port.mapsize = 16;
      isa16550_uart_devices[n].port.port.membase = membase + io[n];
      isa16550_uart_devices[n].port.port.uartclk = 1843200;
#ifdef DEBUG
      printk("register uart at 0x%08X, irq %d\n",
            (unsigned int)isa16550_uart_devices[n].port.port.mapbase,
            isa16550_uart_devices[n].port.port.irq );
#endif
      isa16550_uart_devices[n].num =
         serial8250_register_8250_port(&isa16550_uart_devices[n].port);

      if (isa16550_uart_devices[n].num  < 0)
         pr_err("Failed to register port at 0x%08X\n",
            (unsigned int)res->start + io[n]);

   }

   return 0;
}


static int isa16550_remove(struct platform_device *pdev)
{
   int n;
#ifdef DEBUG
   printk("%s %d\n", __func__, __LINE__);
#endif

   for(n=0; n < NR_UARTS; n++) {
      if (isa16550_uart_devices[n].num  > 0)
         serial8250_unregister_port(isa16550_uart_devices[n].num);
   }

   if (membase)
      devm_iounmap(&pdev->dev, membase);

   return 0;
}


static struct uart_driver isa16550_uart_driver = {
   .owner		= THIS_MODULE,
   .driver_name	= DRIVER_NAME,
   .dev_name	= "ttyZ",
   .nr		= NR_UARTS,
};


module_init(ts7800_isa16550_init);
module_exit(ts7800_isa16550_exit);
MODULE_PARM_DESC(io, "Set offset for uarts (e.g., io=0x2e8,0x3a8,0x2a8,0x3a0)");
module_param_array(io, uint, &io_cnt, 0);
MODULE_PARM_DESC(irq, "Set interrupt numbers for uarts (e.g., irq=5,6,5,6)");
module_param_array(irq, uint, NULL, 0);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Technologic Systems, Inc.");