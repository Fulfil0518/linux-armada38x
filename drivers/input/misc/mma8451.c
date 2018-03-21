/*
 *  Driver for Freescale's 3-Axis Accelerometer MMA8451
 *
 *  Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *
 *  Modified from the MMA8450 driver for the MMA8451 chip
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input-polldev.h>
#include <linux/of_device.h>

#define MMA8451_DRV_NAME	"mma8451"

#define MODE_CHANGE_DELAY_MS	100
#define POLL_INTERVAL		100
#define POLL_INTERVAL_MAX	500

/* register definitions */
#define MMA8451_STATUS		0x00
#define MMA8451_STATUS_ZXYDR	BIT(3)

#define MMA8451_OUT_X_MSB	0x01
#define MMA8451_OUT_X_LSB	0x02
#define MMA8451_OUT_Y_MSB	0x03
#define MMA8451_OUT_Y_LSB	0x04
#define MMA8451_OUT_Z_MSB	0x05
#define MMA8451_OUT_Z_LSB	0x06

#define MMA8451_F_SETUP   0x09

#define MMA8451_WHO_AM_I  0x0D
#define MMA8451_ID        0x1A

#define FF_MT_CFG	        0x15

#define MMA8451_CTRL_REG1	0x2A
#define MMA8451_CTRL_REG2	0x2B

/* mma8451 status */
struct mma8451 {
   struct i2c_client	*client;
   struct input_polled_dev	*idev;
};

static int mma8451_read(struct mma8451 *m, unsigned off)
{
   struct i2c_client *c = m->client;
   int ret;

   ret = i2c_smbus_read_byte_data(c, off);
   if (ret < 0)
      dev_err(&c->dev,
         "failed to read register 0x%02x, error %d\n",
         off, ret);

   return ret;
}

static int mma8451_write(struct mma8451 *m, unsigned off, u8 v)
{
   struct i2c_client *c = m->client;
   int error;

   error = i2c_smbus_write_byte_data(c, off, v);
   if (error < 0) {
      dev_err(&c->dev,
         "failed to write to register 0x%02x, error %d\n",
         off, error);
      return error;
   }

   return 0;
}

static int mma8451_read_block(struct mma8451 *m, unsigned off,
               u8 *buf, size_t size)
{
   struct i2c_client *c = m->client;
   int err;

   err = i2c_smbus_read_i2c_block_data(c, off, size, buf);
   if (err < 0) {
      dev_err(&c->dev,
         "failed to read block data at 0x%02x, error %d\n",
         off, err);
      return err;
   }

   return 0;
}

static void mma8451_poll(struct input_polled_dev *dev)
{
   struct mma8451 *m = dev->private;
   int x, y, z;
   int ret;
   u8 buf[6];

   ret = mma8451_read(m, MMA8451_STATUS);
   if (ret < 0)
      return;

   if (!(ret & MMA8451_STATUS_ZXYDR))
      return;

   ret = mma8451_read_block(m, MMA8451_OUT_X_MSB, buf, sizeof(buf));
   if (ret < 0)
      return;

   x = ((int)(s8)buf[0] << 6) | (0x3f&(buf[1] >> 2));
   y = ((int)(s8)buf[2] << 6) | (0x3f&(buf[3] >> 2));
   z = ((int)(s8)buf[4] << 6) | (0x3f&(buf[5] >> 2));

   input_report_abs(dev->input, ABS_X, x);
   input_report_abs(dev->input, ABS_Y, y);
   input_report_abs(dev->input, ABS_Z, z);
   input_sync(dev->input);
}

/* Initialize the MMA8451 chip */
static void mma8451_open(struct input_polled_dev *dev)
{
   struct mma8451 *m = dev->private;
   int err;

   /* FIFO Mode -> 0 */
   err = mma8451_write(m, MMA8451_F_SETUP, 0);
   if (err)
      return;

   /* enable all events from X/Y/Z, no FIFO */
   err = mma8451_write(m, FF_MT_CFG, BIT(3) | BIT(4) | BIT(5));
   if (err)
      return;

   /*
    * Sleep mode poll rate - 50Hz
    * System output data rate - 400Hz
    * Full scale selection - Active, +/- 2G
    */
   err = mma8451_write(m, MMA8451_CTRL_REG1, BIT(3) | BIT(0));
   if (err < 0)
      return;

   msleep(MODE_CHANGE_DELAY_MS);
}

static void mma8451_close(struct input_polled_dev *dev)
{
   struct mma8451 *m = dev->private;

   mma8451_write(m, MMA8451_CTRL_REG1, 0x00);
   mma8451_write(m, MMA8451_CTRL_REG2, BIT(6));
}

/*
 * I2C init/probing/exit functions
 */
static int mma8451_probe(struct i2c_client *c,
          const struct i2c_device_id *id)
{
   struct input_polled_dev *idev;
   struct mma8451 *m;
   int err, chip_id;

   chip_id = i2c_smbus_read_byte_data(c, MMA8451_WHO_AM_I);
   if (chip_id < 0) {
      dev_err(&c->dev,
         "failed to read register 0x%02X, error %d\n",
         MMA8451_WHO_AM_I, chip_id);
      return -ENODEV;
   }

   if (chip_id != MMA8451_ID) {
       dev_err(&c->dev,
         "Read invalid ID from device register 0x%02X, got 0x%02X\n",
         MMA8451_WHO_AM_I, chip_id);
      return -ENODEV;
   }

   m = devm_kzalloc(&c->dev, sizeof(*m), GFP_KERNEL);
   if (!m)
      return -ENOMEM;

   idev = devm_input_allocate_polled_device(&c->dev);
   if (!idev)
      return -ENOMEM;

   m->client = c;
   m->idev = idev;

   idev->private		= m;
   idev->input->name	= MMA8451_DRV_NAME;
   idev->input->id.bustype	= BUS_I2C;
   idev->poll		= mma8451_poll;
   idev->poll_interval	= POLL_INTERVAL;
   idev->poll_interval_max	= POLL_INTERVAL_MAX;
   idev->open		= mma8451_open;
   idev->close		= mma8451_close;

   __set_bit(EV_ABS, idev->input->evbit);
   __set_bit(INPUT_PROP_ACCELEROMETER, idev->input->propbit);
   input_set_abs_params(idev->input, ABS_X, -8192, 8191, 32, 32);
   input_set_abs_params(idev->input, ABS_Y, -8192, 8191, 32, 32);
   input_set_abs_params(idev->input, ABS_Z, -8192, 8191, 32, 32);

   err = input_register_polled_device(idev);
   if (err) {
      dev_err(&c->dev, "failed to register polled input device\n");
      return err;
   }

   i2c_set_clientdata(c, m);

   return 0;
}

static const struct i2c_device_id mma8451_id[] = {
   { MMA8451_DRV_NAME, 0 },
   { },
};
MODULE_DEVICE_TABLE(i2c, mma8451_id);

static const struct of_device_id mma8451_dt_ids[] = {
   { .compatible = "fsl,mma8451", },
   { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mma8451_dt_ids);

static struct i2c_driver mma8451_driver = {
   .driver = {
      .name	= MMA8451_DRV_NAME,
      .of_match_table = mma8451_dt_ids,
   },
   .probe		= mma8451_probe,
   .id_table	= mma8451_id,
};

module_i2c_driver(mma8451_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MMA8451 3-Axis Accelerometer Driver");
MODULE_LICENSE("GPL");
