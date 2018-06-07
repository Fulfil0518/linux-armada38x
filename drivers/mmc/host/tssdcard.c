#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/module.h>
#include <linux/kallsyms.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/blkdev.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/hdreg.h>
#include <linux/proc_fs.h>
#include <linux/reboot.h>
#include <linux/pci.h>

#define DRIVER_NAME "tssdcard"

/*
 * Current use of this driver only includes TS-7800-V2
 * but TS-7250-V2 could use up to 3
 */
#define MAX_SDS	3

#define SDPEEK8(sd, x) readb((uint32_t *)((sd)->sd_regstart + (x)))
#define SDPOKE8(sd, x, y) writeb(y, (uint32_t *)((sd)->sd_regstart + (x)))
#define SDPEEK16(sd, x) readw((uint32_t *)((sd)->sd_regstart + (x)))
#define SDPOKE16(sd, x, y) writew(y, (uint32_t *)((sd)->sd_regstart + (x)))
#define SDPEEK32(sd, x) readl((uint32_t *)((sd)->sd_regstart + (x)))
#define SDPOKE32(sd, x, y) writel(y, (uint32_t *)((sd)->sd_regstart + (x)))

/* Layer includes low level hardware support */
#include "tssdcore2.c"

static DEFINE_MUTEX(tssdcore_lock);

struct tssdcard_dev {
	struct device *dev;
	struct sdcore tssdcore;
	char *devname;
	sector_t sectors;
	struct gendisk *gd;
	struct request_queue *queue;
	struct list_head tssdcard_deferred;
	spinlock_t lock;
	struct work_struct tssdcard_transfer_work;
	struct workqueue_struct *tssdcard_transfer_queue;
	struct work_struct diskpoll_work;
	struct workqueue_struct *diskpoll_queue;
	struct timer_list cd_timer;
	unsigned long timeout;
	unsigned long lasttimeout;
	int cardpresent;
	int lasterr;
	int major;
};

struct tssdcard_host {
	struct platform_device *pdev;
	struct resource *mem_res;
	void __iomem *base;
	int numluns;
	struct tssdcard_dev luns[MAX_SDS];
};

void tssdcard_debug(void *arg,
		    unsigned int code,
		    const char *func,
		    unsigned int line, ...)
{
	struct tssdcard_dev *dev = (struct tssdcard_dev *)arg;
	va_list ap;
	unsigned int s, x, y, z;

	/* Only print each message once */
	if (dev->lasterr == code)
		return;
	dev->lasterr = code;

	va_start(ap, line);
	switch (code) {
	case SD_HW_TMOUT:
		s = va_arg(ap, unsigned int); /* sector */
		x = va_arg(ap, unsigned int); /* reg val */
		pr_info("SD hardware timeout, sect=%u (0x%x)\n", s, x);
		break;
	case SD_DAT_BAD_CRC:
		s = va_arg(ap, unsigned int); /* sector */
		x = va_arg(ap, unsigned int); /* reg val */
		pr_info("SD hw detected bad CRC16, sect=%u (0x%x)\n", s, x);
		break;
	case READ_FAIL:
		s = va_arg(ap, unsigned int); /* sector */
		pr_info("SD read failed, sect=%u\n", s);
		break;
	case WRITE_FAIL:
		s = va_arg(ap, unsigned int); /* sector */
		x = va_arg(ap, unsigned int); /* sdcmd() ret status */
		pr_info("SD write failed, sect=%u (0x%x)\n", s, x);
		break;
	case SD_STOP_FAIL:
		x = va_arg(ap, unsigned int); /* sdcmd() ret status */
		pr_info("SD stop transmission failed (0x%x)\n", x);
		break;
	case SD_RESP_FAIL:
		x = va_arg(ap, unsigned int); /* SD cmd */
		y = va_arg(ap, unsigned int); /* response status */
		pr_info("SD cmd 0x%x resp code has err bits 0x%x\n", x, y);
		break;
	case SD_RESP_BAD_CRC:
		x = va_arg(ap, unsigned int); /* SD cmd */
		y = va_arg(ap, unsigned int); /* calculated */
		z = va_arg(ap, unsigned int); /* rx'ed */
		pr_info("SD cmd 0x%x resp bad crc (0x%x != 0x%x)\n", x, y, z);
		break;
	case SD_RESP_WRONG_REQ:
		x = va_arg(ap, unsigned int); /* SD cmd */
		y = va_arg(ap, unsigned int); /* cmd in response */
		pr_info("SD response for wrong cmd. (0x%x != 0x%x)\n", x, y);
	case SD_SW_TMOUT:
		if (dev->cardpresent)
			pr_info("SD soft timeout(%pS)\n",
				__builtin_return_address(0));
		break;
	}
	va_end(ap);
}

static void tssdcard_delay(void *data, unsigned int us)
{
	if (us > 50000)
		msleep_interruptible(us/1000);
	else
		udelay(us);
}

static int tssdcard_timeout_relaxed(void *data)
{
	struct sdcore *sd = (struct sdcore *)data;
	struct tssdcard_dev *dev = (struct tssdcard_dev *)sd->os_arg;
	int ret;

	dev->lasttimeout = jiffies;

	if (jiffies_to_msecs(dev->timeout - jiffies) > 50)
		msleep_interruptible(10);

	ret = time_is_before_jiffies(dev->timeout);

	return ret;
}

static int tssdcard_timeout(void *data)
{
	struct sdcore *sd = (struct sdcore *)data;
	struct tssdcard_dev *dev = (struct tssdcard_dev *)sd->os_arg;
	int ret;

	dev->lasttimeout = jiffies;
	ret = time_is_before_jiffies(dev->timeout);

	return ret;
}

static void tssdcard_reset_timeout(void *data)
{
	struct sdcore *sd = (struct sdcore *)data;
	struct tssdcard_dev *dev = (struct tssdcard_dev *)sd->os_arg;

	/* SD Spec allows 1 second for cards to answer */
	dev->timeout = jiffies + HZ;
}

static void tssdcard_irqwait(void *data, unsigned int x)
{
	struct tssdcard_dev *dev = (struct tssdcard_dev *)data;
	uint32_t reg;

	do {
#ifdef CONFIG_PREEMPT_NONE
		/* Default marvell kernel config has no preempt, so
		 * to support that:
		 */
		cond_resched();
#endif
		reg = readl((uint32_t *)(dev->tssdcore.sd_regstart + 0x108));
	} while ((reg & 4) == 0);
}

static int tssdcard_transfer(struct tssdcard_dev *dev, unsigned long sector,
			      unsigned long nsect, char *buffer, int rw)
{
	int ret = 0;

	dev_dbg(dev->dev, "%s size:%lld sector:%lu nsect:%lu rw:%d\n",
		__func__, (long long)dev->sectors, sector, nsect, rw);

	tssdcard_reset_timeout(&dev->tssdcore);
	switch (rw) {
	case WRITE:
		ret = sdwrite(&dev->tssdcore, sector, buffer, nsect);
		if (ret && !dev->tssdcore.sd_wprot) {
			if (sdreset(&dev->tssdcore) != 0) {
				tssdcard_reset_timeout(dev);
				ret = sdwrite(&dev->tssdcore, sector,
					      buffer, nsect);
			}
		}
		break;

	case READ:
	case READA:
		ret = sdread(&dev->tssdcore, sector, buffer, nsect);

		if (ret) {
			if (sdreset(&dev->tssdcore) != 0) {
				tssdcard_reset_timeout(dev);
				ret = sdread(&dev->tssdcore, sector,
					     buffer, nsect);
			}
		}
	}

	return ret;
}

static void tssdcard_request(struct request_queue *rq)
{
	struct request *req;
	struct tssdcard_dev *dev = rq->queuedata;

	while ((req = blk_fetch_request(rq)) != NULL) {
		/*
		 * Add to list of deferred work and then schedule
		 * workqueue.
		 */
		list_add_tail(&req->queuelist, &dev->tssdcard_deferred);
		queue_work(dev->tssdcard_transfer_queue,
			   &dev->tssdcard_transfer_work);
	}
}

static int tssdcard_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	struct tssdcard_dev *dev = bdev->bd_disk->private_data;

	geo->cylinders = dev->sectors >> 9 / (4 * 16);
	geo->heads = 4;
	geo->sectors = 16;
	return 0;
}

static const struct block_device_operations tssdcard_ops = {
	.owner			= THIS_MODULE,
	.getgeo			= tssdcard_getgeo,
};

static void tssdcard_alloc_disk(struct tssdcard_dev *dev)
{
	dev->gd = alloc_disk(CONFIG_MMC_BLOCK_MINORS);
	if (dev->gd == NULL) {
		pr_err(DRIVER_NAME ": Failed to alloc_disk");
		return;
	}

	strcpy(dev->gd->disk_name, dev->devname);

	dev->queue = blk_init_queue(tssdcard_request, &dev->lock);
	if (dev->queue == NULL) {
		pr_err(DRIVER_NAME ": Failed to init blk queue");
		return;
	}
	blk_queue_logical_block_size(dev->queue, 512);
	queue_flag_set_unlocked(QUEUE_FLAG_NONROT, dev->queue);
	dev->queue->queuedata = dev;
	set_capacity(dev->gd, dev->sectors);
	dev->gd->major = dev->major;
	dev->gd->first_minor = dev->tssdcore.sd_lun * CONFIG_MMC_BLOCK_MINORS;
	dev->gd->flags = 0;
	dev->gd->fops = &tssdcard_ops;
	dev->gd->queue = dev->queue;
	dev->gd->private_data = dev;
	add_disk(dev->gd);
}

static void tssdcard_transfer_thread(struct work_struct *work)
{
	struct tssdcard_dev *dev = container_of(work, struct tssdcard_dev,
		tssdcard_transfer_work);
	struct list_head *elem, *next;
	struct request *req;
	unsigned int start;
	unsigned int len;
	int err;

	if (list_empty(&dev->tssdcard_deferred))
		return;

	spin_lock(&dev->lock);
	list_for_each_safe(elem, next, &dev->tssdcard_deferred) {
		req = list_entry(elem, struct request, queuelist);
		spin_unlock(&dev->lock);

		err = 0;
		start = blk_rq_pos(req);
		len = blk_rq_cur_sectors(req);

		/* Make sure this write isn't past end of disk */
		if (start + len > (dev->sectors)) {
			pr_err("Access past EOD, block=%llu, count=%u\n",
				(unsigned long long)blk_rq_pos(req),
				blk_rq_cur_sectors(req));
			err = -EIO;
			len = 0;
		}

		if (!dev->cardpresent) {
			err = -ENOMEDIUM;
			len = 0;
		}

		mutex_lock(&tssdcore_lock);
		while (len) {
			if (tssdcard_transfer(dev, blk_rq_pos(req),
			    blk_rq_cur_sectors(req), bio_data(req->bio),
			    rq_data_dir(req))) {
				mutex_unlock(&tssdcore_lock);

				spin_lock(&dev->lock);
				list_del_init(&req->queuelist);
				__blk_end_request_all(req, -ENOMEDIUM);
				spin_unlock(&dev->lock);

				/* Failing SD reads/writes is how we
				 * normally detect a card that has been
				 * removed.  We can't destroy the queue
				 * from this callback, so start the
				 * diskpoll thread to handle this.
				 */
				dev->cardpresent = 0;
				queue_work(dev->diskpoll_queue,
					   &dev->diskpoll_work);
				return;
			}
			len--;
		}
		mutex_unlock(&tssdcore_lock);
		spin_lock(&dev->lock);
		list_del_init(&req->queuelist);
		__blk_end_request_all(req, err);
	}

	spin_unlock(&dev->lock);
}

static void diskpoll_thread(struct work_struct *work)
{
	struct tssdcard_dev *dev = container_of(work, struct tssdcard_dev,
		diskpoll_work);

	if (!dev->cardpresent && dev->sectors != 0) {
		pr_info("SD card was removed!\n");
		mutex_lock(&tssdcore_lock);
		cancel_work_sync(&dev->tssdcard_transfer_work);
		mutex_unlock(&tssdcore_lock);
		del_gendisk(dev->gd);
		blk_cleanup_queue(dev->queue);
		put_disk(dev->gd);
		dev->sectors = 0;
	} else {
		mutex_lock(&tssdcore_lock);
		dev->sectors = sdreset(&dev->tssdcore);
		mutex_unlock(&tssdcore_lock);
	}

	if (dev->sectors == 0) {
		dev->tssdcore.os_timeout = tssdcard_timeout_relaxed;
		mod_timer(&dev->cd_timer, jiffies + HZ);
	} else {
		dev->cardpresent = 1;
		dev->tssdcore.os_timeout = tssdcard_timeout;
		tssdcard_alloc_disk(dev);
	}
}

static void tssdcard_card_poll(unsigned long data)
{
	struct tssdcard_dev *dev = (struct tssdcard_dev *) data;

	queue_work(dev->diskpoll_queue, &dev->diskpoll_work);
}

static int setup_device(struct tssdcard_host *host, int lun)
{
	int ret = 0;
	struct tssdcard_dev *dev = &host->luns[lun];

	dev->dev = &host->pdev->dev;
	/* IO Remapping (use the same virtual address for all LUNs) */
	dev->tssdcore.sd_regstart = (unsigned int)host->base;
	dev->tssdcore.sd_lun = lun;

	//dev and tssdcore struct initialization
	dev->tssdcore.os_timeout = tssdcard_timeout;
	dev->tssdcore.os_reset_timeout = tssdcard_reset_timeout;
	dev->tssdcore.os_arg = dev;
	dev->tssdcore.os_delay = tssdcard_delay;
	dev->tssdcore.os_irqwait = tssdcard_irqwait;
	dev->tssdcore.sd_writeparking = 1;
	dev->tssdcore.debug = tssdcard_debug;
	dev->tssdcore.debug_arg = dev;
	dev->major = register_blkdev(UNNAMED_MAJOR, DRIVER_NAME);
	INIT_LIST_HEAD(&dev->tssdcard_deferred);

	dev->devname = kmalloc(32, GFP_KERNEL);
	if (!dev->devname)
		return -ENOMEM;
	snprintf(dev->devname, 32, "%s%c", DRIVER_NAME, lun + 'a');

	spin_lock_init(&dev->lock);

	/* sdreset sleeps so we need our own workqueue */
	dev->diskpoll_queue = alloc_ordered_workqueue(dev->devname, 0);
	if (!dev->diskpoll_queue)
		return -ENOMEM;

	dev->tssdcard_transfer_queue = alloc_ordered_workqueue(dev->devname, 0);
	if (!dev->tssdcard_transfer_queue)
		return -ENOMEM;

	INIT_WORK(&dev->diskpoll_work, diskpoll_thread);
	INIT_WORK(&dev->tssdcard_transfer_work, tssdcard_transfer_thread);

	dev->cd_timer.function = tssdcard_card_poll;
	dev->cd_timer.data = (unsigned long)dev;

	/* Start polling for the card */
	queue_work(dev->diskpoll_queue,
	   &dev->diskpoll_work);

	return ret;
}

static int tssdcard_probe(struct platform_device *pdev)
{
	int i, ret = 0;
	struct tssdcard_host *host;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res = 0;
	struct pci_dev *pcidev;

	host = kzalloc(sizeof(struct tssdcard_host), GFP_KERNEL);
	if (host == NULL) {
		ret = -ENOMEM;
		goto out;
	}

	/* The pcie must be enabled before we can access the fpga registers! */
	pcidev = pci_get_device(0x1204, 0x0001, NULL);
	if (!pcidev) {
		pr_err("Cannot find FPGA at PCI 1204:0001\n");
		ret = -EINVAL;
		goto out;
	}

	if (pci_enable_device(pcidev)) {
		pr_err("Cannot enable FPGA at PCI 1204:0001\n");
		ret = -EINVAL;
		goto out;
	}

	if (of_property_read_u32(np, "tssdcard,ndevices", &host->numluns) < 0) {
		pr_info("Can't read property 'tssdcard,ndevices' in device-tree; assuming 1\n");
		host->numluns = 1;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!devm_request_mem_region(&pdev->dev, res->start,
	    resource_size(res), pdev->name)) {
		ret = -EBUSY;
		goto out;
	}

	host->base = devm_ioremap_nocache(&pdev->dev, res->start,
					  resource_size(res));
	if (!host->base) {
		ret = -EFAULT;
		goto out;
	}

	for (i = 0; i < host->numluns; i++) {
		ret = setup_device(host, i);
		if (ret)
			goto out;
	}

	platform_set_drvdata(pdev, host);

	return 0;

out:
	return ret;
}

static int tssdcard_remove(struct platform_device *pdev)
{
	struct tssdcard_host *host = (struct tssdcard_host *)pdev->dev.p;
	int i;

	for (i = 0; i < host->numluns; i++) {
		struct tssdcard_dev *dev = &host->luns[i];

		dev_dbg(dev->dev, "dev[%d] ...\n", i);

		if (dev->sectors == 0)
			continue;

		if (dev->gd)
			put_disk(dev->gd);

		kfree(dev->devname);

		if (dev->queue) {
			blk_cleanup_queue(dev->queue);
			blk_put_queue(dev->queue);
		}
	}
	return 0;
}

static const struct platform_device_id tssdcard_devtype[] = {
	{
		.name = "tssdcard-mmc",
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, tssdcard_devtype);

static const struct of_device_id tssdcard_of_match[] = {
	{
		.compatible = "technologicsystems,tssdcard",
	}
};

static struct platform_driver tssdcard_driver = {
	.probe =  tssdcard_probe,
	.remove = tssdcard_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = tssdcard_of_match,
	}
};

module_platform_driver(tssdcard_driver);

MODULE_DESCRIPTION("TS-7800-V2 SDHC Driver");
MODULE_AUTHOR("Ian Coughlan, Technologic Systems");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tssdcard");
