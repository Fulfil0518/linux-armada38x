/*
 * Copyright (C) Technologic Systems, Inc.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/i2c.h>


#define DEVNAME "ts7800v2-wdt"

static struct i2c_client *i2c_client;

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
            __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static int ts7800v2_wdt_write(unsigned int val)
{
   unsigned char data[6];

   if (val == 0) {
      data[0] = 0x04; data[1] = 0x04; /* reg address is 0x404, or 1028 decimal*/
      *(unsigned int*)&data[2] = 0;

   } else {
      data[0] = 0x04; data[1] = 0; /* reg address is 0x400, or 1024 decimal */
      *(unsigned int*)&data[2] = val * 100;
   }

   return i2c_master_send(i2c_client, data, sizeof(data));
}

static int ts7800v2_wdt_start(struct watchdog_device *wdt)
{
   unsigned char data[6] = {
      0x04,0x04,  /* reg address is 0x404, or 1028 decimal */
      1,0,0,0     /* Write 1 to enable the wdt */
   };

   ts7800v2_wdt_write(wdt->timeout + 1);
   return i2c_master_send(i2c_client, data, sizeof(data));
}

static int ts7800v2_wdt_stop(struct watchdog_device *wdt)
{
   return ts7800v2_wdt_write(0);
}

static int ts7800v2_wdt_ping(struct watchdog_device *wdt)
{
  unsigned char data[6] = {
      0x04,0x04,  /* reg address is 0x404, or 1028 decimal */
      1,0,0,0     /* Write 1 to enable/feed the wdt */
  };

  return i2c_master_send(i2c_client, data, sizeof(data));
}


static long ts7800v2_wdt_ioctl(struct watchdog_device *wdt, unsigned int cmd, unsigned long param)
{
   if (cmd == WDIOC_KEEPALIVE)
      return ts7800v2_wdt_ping(wdt);
   else
      return -ENOIOCTLCMD;
}

static int ts7800v2_wdt_set_timeout(struct watchdog_device *wdt,
               unsigned int timeout)
{
   wdt->timeout = timeout;
   return 0;
}

static const struct watchdog_info ts7800v2_wdt_info = {
   .options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
   .identity = DEVNAME,
};

static const struct watchdog_ops ts7800v2_wdt_ops = {
   .owner		= THIS_MODULE,
   .start		= ts7800v2_wdt_start,
   .stop		= ts7800v2_wdt_stop,
   .ping = ts7800v2_wdt_ping,
   .set_timeout	= ts7800v2_wdt_set_timeout,
   .ioctl = ts7800v2_wdt_ioctl,
};


static int ts7800v2_wdt_probe(struct i2c_client *client,
               const struct i2c_device_id *id)
{
   int ret = 0;
   struct watchdog_device *wdt;

   i2c_client = client;

   wdt = devm_kzalloc(&client->dev, sizeof(*wdt), GFP_KERNEL);
   if (!wdt)
      return -ENOMEM;

   wdt->info	= &ts7800v2_wdt_info;
   wdt->ops		= &ts7800v2_wdt_ops;
   wdt->status		= 0;
   wdt->timeout		= 30;
   wdt->min_timeout	= 1;
   wdt->max_timeout	= 30;
   wdt->parent = &client->dev;

   watchdog_set_nowayout(wdt, nowayout);
   i2c_set_clientdata(client, wdt);
   ts7800v2_wdt_stop(wdt);

   ret = watchdog_register_device(wdt);
   if (ret)
      return ret;

   ts7800v2_wdt_start(wdt);

   return 0;
}

static int ts7800v2_wdt_remove(struct i2c_client *client)
{
   struct watchdog_device *wdt = i2c_get_clientdata(client);

   watchdog_unregister_device(wdt);   return 0;
}


static const struct of_device_id ts7800v2_wdt_of_match[] = {
   { .compatible = "technologicsystems,ts7800v2-wdt", },
   { },
};
MODULE_DEVICE_TABLE(of, ts7800v2_wdt_of_match);

static const struct i2c_device_id ts7800v2_wdt_id[] = {
   {DEVNAME, 0},
   {},
};

MODULE_DEVICE_TABLE(i2c, ts7800v2_wdt_id);

static struct i2c_driver ts7800v2_wdt_driver = {
   .probe		= ts7800v2_wdt_probe,
   .remove		= ts7800v2_wdt_remove,
   .id_table = ts7800v2_wdt_id,
   .driver		= {
      .name		= DEVNAME,
      .of_match_table	= ts7800v2_wdt_of_match,
   },
};

module_i2c_driver(ts7800v2_wdt_driver);

MODULE_AUTHOR("Technologic Systems Inc");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ts7800v2-wdt");

