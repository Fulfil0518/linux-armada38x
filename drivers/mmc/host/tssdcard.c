#include <linux/module.h>
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

struct tssdcard_host {
   struct platform_device *pdev;
   void __iomem      *base;
   dma_addr_t     phys_base;
   struct resource *res, *mem_res;
   int irq;
} ts_host;

#include "tssdcore2.c"

enum reqmode_t {
  RM_SIMPLE  = 0,  /* The extra-simple request function */
  RM_FULL    = 1,  /* The full-blown version */
  RM_NOQUEUE = 2,  /* Use make_request */
};

#define DMA_ENABLE      0
#define THREAD_ENABLE   1
#define DMA_CHANNEL     0
#define REQUEST_MODE    RM_NOQUEUE  /* The only one that currently works! */
#define DEBUG_ENABLE    0
#define NDEVICES       1  /* TS-7800-V2 ndevices=1 for Micro and Full size sockets */

#define SD_DEFIOSIZE    4
#define KERNEL_SECTOR_SIZE 512
#define SD_DEFIOSIZE    4
#define SD_BLKSIZE      1024  //nsectors?
#define SD_HARDSEC      512
#define SD_SHIFT     4  //max 16 partitions 2^4
#define SD_NUMPART      (1 << 4)
#define TSSDCARD_MINORS    SD_NUMPART
#define MINOR_SHIFT     SD_SHIFT
#define DEVNUM(kdevnum)    (MINOR(kdev_t_to_nr(kdevnum)) >> MINOR_SHIFT


#define TS7800_IRQ_DMA     65
#define TS7800_IRQ_WAIT    66
#define TS7XXX_SDCARD1     0xE8000100
#define TS7800_DMAADDR     0xE8000400
#define TS7800_FPGASDDAT   0x80000104
#define TS7800_CPUDOORBELL    (ORION5X_REGS_VIRT_BASE + 0x20400) /*0xF1020400*/
#define TS7800_FPGADOORBELL   0xE8000200
#define TS7800_FPGAMSIMASK    0xE8000204
#define TS7800_FPGAIRQRAW     0xE8000208
#define DOORBELL_DMABIT    (0x1 << 1)
#define DOORBELL_WAITBIT   (0x1 << 2)

#define blk_fs_request(rq) ((rq)->cmd_type == REQ_TYPE_FS)

#define my_inb(x) readb(x)
#define my_outb(x,y) writeb(x,y)
#define my_inw(x) readw(x)
#define my_outw(x,y) writew(x,y)
#define my_inl(x) readl(x)
#define my_outl(x,y) writel(x,y)

struct tssdcard_dev {
  struct sdcore tssdcore;
  char *devname;
  sector_t size;        /* Device size in sectors */
  struct gendisk *gd;      /* The gendisk structure */
  //struct timer_list timer;
  short media_change;      /* Flag a media change? */
  int irq, irq_wait;
  unsigned long dmaregs;
  unsigned long sysconfpga;
  int dmach;
  atomic_t users;    /* How many users */
  sector_t parksect;
  struct request_queue *queue;   /* The device request queue */
  struct bio *bio;
  struct bio *biotail;
  struct task_struct *thread;
  wait_queue_head_t event;
  spinlock_t lock;

  int write_activity;
};

static atomic_t busy;
static struct semaphore sem;
static wait_queue_head_t qwait;
static struct tssdcard_dev *devices = NULL;
static int ndevices;
static enum reqmode_t reqmode = REQUEST_MODE;
static int threnable = THREAD_ENABLE;
static int dbgenable = DEBUG_ENABLE;
static int tssdcard_major = 0;
static int dmach = DMA_CHANNEL;

static const struct platform_device_id tssdcard_devtype[] = {
   {
      .name = "tssdcard-mmc",
      .driver_data = (kernel_ulong_t)&ts_host,
   }, {
      /* sentinel */
   }
};
MODULE_DEVICE_TABLE(platform, tssdcard_devtype);


static const struct of_device_id tssdcard_of_match[] = {
   {
      .compatible = "technologicsystems,tssdcard",
      .data = &tssdcard_devtype[0],
   }
};

static void tssdcard_handle_bio(struct tssdcard_dev *dev, struct bio *bio);
static void tssdcard_handle_simplereq(struct tssdcard_dev *dev);
static void tssdcard_handle_fullreq(struct tssdcard_dev *dev);
static void tssdcard_full_request(struct request_queue *q);
static void tssdcard_add_bio(struct tssdcard_dev *dev, struct bio *bio);
static struct bio *tssdcard_get_bio(struct tssdcard_dev *dev);
static void tssdcard_transfer(struct tssdcard_dev *dev, unsigned long sector,
    unsigned long nsect, char *buffer, int rw);


static void tssdcard_delay(void *arg, unsigned int us)
{
  while(us > 1000) {
   mdelay(1);
   us -= 1000;
 }

 if (us > 0)
   udelay(us);
}


static void handle_request(struct request_queue *q, struct bio *bio)
{
  struct tssdcard_dev *dev = q->queuedata;

  if (reqmode != RM_NOQUEUE) spin_unlock_irq(&dev->lock);

  if (threnable) {
    if (reqmode == RM_NOQUEUE && bio)
      tssdcard_add_bio(dev, bio);
    wake_up(&dev->event);
  }
  else {
    if (down_interruptible(&sem))
      return; // -ERESTARTSYS;
    atomic_inc(&busy);
    if (atomic_read(&busy) > 1)
      panic("recursive make_request!\n");

    if (reqmode == RM_NOQUEUE) {
      if(bio) tssdcard_handle_bio(dev, bio);
    } else if (reqmode == RM_FULL)
      tssdcard_handle_fullreq(dev);
    else if (reqmode == RM_SIMPLE)
      tssdcard_handle_simplereq(dev);

    atomic_dec(&busy);
    up(&sem);
  }

  if (reqmode != RM_NOQUEUE) spin_lock_irq(&dev->lock);

  return;

}


/* Request Mode RM_SIMPLE=0 */
static void tssdcard_request(struct request_queue *q)
{
   handle_request(q, NULL);
}

/* Request Mode RM_FULL=1 Clustering */
static void tssdcard_full_request(struct request_queue *q)
{
  return handle_request(q, NULL);
}

/* Request Mode RM_NOQUEUE=2 */
static blk_qc_t tssdcard_make_request(struct request_queue *q, struct bio *bio)
{
  handle_request(q, bio);
  return 0;
}


static void tssdcard_handle_bio(struct tssdcard_dev *dev, struct bio *bio)
{
  struct bio_vec bvec;
  struct bvec_iter iter;
  sector_t sector, end_sector, n_sectors;
  char *buffer;

  if(dbgenable) printk(KERN_NOTICE "%s %d\n", __func__, __LINE__);

  sector = bio->bi_iter.bi_sector;
  end_sector = (bio->bi_iter.bi_sector) + (bio->bi_iter.bi_size >> 9) - 1;

  if((bio->bi_iter.bi_size % 512) != 0)
    panic("Invalid transfer, bi_size 512 != 0\n");

  bio_for_each_segment(bvec, bio, iter) {
    if((sector + (bvec.bv_len >> 9)) > end_sector)
        n_sectors = end_sector - sector + 1;
    else
       n_sectors = bvec.bv_len >> 9;
    if(n_sectors == 0) continue;

    buffer = kmap(bvec.bv_page) + bvec.bv_offset;
    tssdcard_transfer(dev, sector, n_sectors, buffer, bio_data_dir(bio));
    sector += n_sectors;
    kunmap(bvec.bv_page);
  }

  bio_endio(bio);

  return;
}


/*
 * Handle an I/O request.
 */
static void tssdcard_transfer(struct tssdcard_dev *dev, unsigned long sector,
    unsigned long nsect, char *buffer, int rw)
{
   int ret;

   if(dbgenable) printk("%s size:%lld sector:%lu nsect:%lu rw:%d\n",
      __func__, (long long)dev->size, sector, nsect, rw);

   if ( (sector + nsect) > (dev->size*512/KERNEL_SECTOR_SIZE) ) {
      printk(KERN_NOTICE "Beyond-end write (%ld %ld)\n", sector, nsect);
      return;
   }

   //del_timer(&dev->timer);
   switch (rw) {
   case WRITE:
    dev->write_activity = 1;

    ret = sdwrite(&dev->tssdcore, sector, buffer, nsect);
    if (ret && !dev->tssdcore.sd_wprot) {
      sdreset(&dev->tssdcore);
      ret = sdwrite(&dev->tssdcore, sector, buffer, nsect);
    }
    //if (ret == 0) mark_buffer_uptodate(buffer, 1);
    //dev->timer.expires = jiffies + (HZ / 2);
    //add_timer(&dev->timer);
    break;

  case READ:
  case READA:
    ret = sdread(&dev->tssdcore, sector, buffer, nsect);
    if (ret) {
      sdreset(&dev->tssdcore);
      ret = sdread(&dev->tssdcore, sector, buffer, nsect);
    }
    dev->parksect = sector + nsect - 1;

    //sdread will park at sector+nsect+1, if we read the last sector
    //this means we are parked on an invalid sector so reset the card.
    if ( (sector + nsect) >= (dev->size*512/KERNEL_SECTOR_SIZE) )
      sdreset(&dev->tssdcore);

    break;
  }

}

static void tssdcard_handle_simplereq(struct tssdcard_dev *dev)
{
   struct request *req;
   struct bio_vec *bvec;
   char *buffer;

   if(dbgenable) printk(KERN_NOTICE "%s %d\n", __func__, __LINE__);

   while ((req = blk_peek_request(dev->queue)) != NULL) {
      if (! blk_fs_request(req)) {
         printk (KERN_NOTICE "Skip non-fs request\n");
         blk_end_request_cur(req, 0);
         continue;
      }
      bvec = req->bio->bi_io_vec;
      buffer = kmap(bvec->bv_page) + bvec->bv_offset;
      tssdcard_transfer(dev, blk_rq_pos(req), blk_rq_cur_sectors(req),
         buffer, rq_data_dir(req));
      kunmap(bvec->bv_page);

      /*
      if (!end_that_request_first(req, 1, blk_rq_cur_sectors(req))) {
         blk_start_request(req);
         end_that_request_last(req, 1);
      }
      */

      __blk_end_request(req, 1, blk_rq_cur_sectors(req) << 9);
   }
}


static void tssdcard_handle_fullreq(struct tssdcard_dev *dev)
{
   unsigned long flags;
   struct request *req;
   struct bio *bio;
   struct bio_vec bvec;

   int sectors_xferred;
   sector_t sector, end_sector, n_sectors;
   char *buffer;
   struct req_iterator iter;
   size_t size;

   end_sector = 0;

   if(dbgenable) printk(KERN_NOTICE "%s %d\n", __func__, __LINE__);

   spin_lock_irqsave(dev->queue->queue_lock, flags);

   while ((req = blk_peek_request(dev->queue))) {
      if (! blk_fs_request(req)) {
         printk(KERN_NOTICE "Skip non-fs request\n");
         blk_end_request_cur(req, 0);
         continue;
      }
      bio = NULL;
      sector = 0;
      sectors_xferred = 0;
      rq_for_each_segment(bvec, req, iter) {
         size = bvec.bv_len;
         if(dbgenable) printk("size: %d\n", size);
         if (bio != iter.bio)   {  // next bio
            bio = iter.bio;
            if (bio == NULL) {
               if(dbgenable) printk("bio is null\n");
               n_sectors = 0;
               break;
            }
            sector = bio->bi_iter.bi_sector;
            end_sector = (bio->bi_iter.bi_sector) + (bio->bi_iter.bi_size >> 9) - 1;
            if(dbgenable) printk("bio: 0x%08lX sector %lld, end_sector %lld, bi_size %d\n", (unsigned long)bio, sector, end_sector, bio->bi_iter.bi_size);
            if((bio->bi_iter.bi_size % 512) != 0)
               panic("Invalid transfer, bi_size 512 != 0\n");
         }

         if((sector + (bvec.bv_len >> 9)) > end_sector)
            n_sectors = end_sector - sector + 1;
         else
            n_sectors = bvec.bv_len >> 9;

         if(n_sectors == 0) {
            if(dbgenable) printk("n_sectors == 0\n");
               continue;
         }

         if(dbgenable) printk("kmap()...\n");
         buffer = kmap(bvec.bv_page);
         if(dbgenable) printk("kmap() 0x%08lX\n", (unsigned long)buffer);

         if (buffer) {
            if(dbgenable) printk("unlocking\n");
            spin_unlock_irqrestore(dev->queue->queue_lock, flags);
            if(dbgenable) printk("unlocked\n");
            tssdcard_transfer(dev, sector, n_sectors, buffer + bvec.bv_offset, bio_data_dir(iter.bio));
            if(dbgenable) printk("locking\n");
            spin_lock_irqsave(dev->queue->queue_lock, flags);
            if(dbgenable) printk("locked\n");
            sector += n_sectors;
            kunmap(bvec.bv_page);
         }
         else
            printk("Error in tssdcard_handle_fullreq(), kmap() returned NULL\n");
      } /* rq_for_each_segment(bvec, req, iter) ... */

      blk_start_request(req);
      __blk_end_request_all(req, 0);
  }
  if(dbgenable) printk("unlocking\n");
  spin_unlock_irqrestore(dev->queue->queue_lock, flags);
  if(dbgenable) printk("unlocked...and done!\n");
}

static void tssdcard_add_bio(struct tssdcard_dev *dev, struct bio *bio)
{
  spin_lock(&dev->lock);
  if (dev->biotail) {
    dev->biotail->bi_next = bio;
    dev->biotail = bio;
  } else
  dev->bio = dev->biotail = bio;
  spin_unlock(&dev->lock);
}

static struct bio *tssdcard_get_bio(struct tssdcard_dev *dev)
{
  struct bio *bio;
  spin_lock(&dev->lock);
  if ((bio = dev->bio)) {
    if (bio == dev->biotail)
      dev->biotail = NULL;
    dev->bio = bio->bi_next;
    bio->bi_next = NULL;
  }
  spin_unlock(&dev->lock);
  return bio;
}

static void tssdcard_release(struct gendisk *disk, fmode_t mode)
{
  struct tssdcard_dev *dev = disk->private_data;

  atomic_dec(&dev->users);
  if (atomic_read(&dev->users) == 0) {
    if (dev->thread != NULL) {
      kthread_stop(dev->thread);
      dev->thread = NULL;
    }
  }
}

#define TSSDCARD_IOC_MAGIC 't'
#define TSSDCARD_GET_WRITE_ACTIVITY _IOR(TSSDCARD_IOC_MAGIC, 1, int)
#define TSSDCARD_GET_SECTOR_COUNT   _IOR(TSSDCARD_IOC_MAGIC, 2, int)


/*
 * The ioctl() implementation
 */

// int tssdcard_ioctl(struct inode *inode, struct file *filp,
//    unsigned int cmd, unsigned long arg)

int tssdcard_ioctl(struct block_device *bdev, fmode_t mode, unsigned int cmd,
          unsigned long arg)
{
  struct hd_geometry geo;

  // Ian added
  struct tssdcard_dev *dev = bdev->bd_disk->private_data;

  switch(cmd) {
    case HDIO_GETGEO:
   /*
         * get geometry: we have to fake one...  trim the size to a
         * multiple of 512 (256k): tell we have 32 sectors, 16 heads,
         * whatever cylinders.
         */

        // TODO: This should match what the SD cards look like in a
        // USB adapter dongle.  -- JO

      geo.cylinders = dev->size / (16 * 32);
      geo.heads = 16;
      geo.sectors = 32;
      geo.start = 0;
      if (copy_to_user((void __user *) arg, &geo, sizeof(geo)))
        return -EFAULT;
      return 0;

  // Ian added
  case TSSDCARD_GET_WRITE_ACTIVITY:
    {
       if (dev==NULL)
       {
          printk("Error: dev is NULL in tssdcard\n");
          return -EFAULT;
       }

       if (copy_to_user((void __user *) arg, &dev->write_activity, sizeof(dev->write_activity)))
        return -EFAULT;
      dev->write_activity = 0;
      return 0;
    }

    // Ian added
    case TSSDCARD_GET_SECTOR_COUNT:
       if (dev==NULL)
       {
          printk("Error: dev is NULL in tssdcard\n");
          return -EFAULT;
       }

       if (copy_to_user((void __user *) arg, &dev->size, sizeof(dev->size)))
        return -EFAULT;

      return 0;


    //default:
      /*
       * For ioctls we don't understand, let the block layer handle them.
       */
      //return blkdev_ioctl(inode, filp, cmd, arg);

  }

  return -ENOTTY; /* unknown command */
}


int tssdcard_media_changed(struct gendisk *gd)
{
  struct tssdcard_dev *dev;

  char buf[512];
  int ret;

  dev = gd->private_data;
  if (dev == NULL) {
   printk("%s %d oops, gd->dev is NULL\n", __func__, __LINE__);
   return 0;
  }

  if (down_interruptible(&sem)) return -ERESTARTSYS;
  ret = sdread(&dev->tssdcore, 1, buf, 1);
  up(&sem);
  return ret;
}

int tssdcard_revalidate(struct gendisk *gd)
{
  struct tssdcard_dev *dev;

  dev = gd->private_data;
  if (dev == NULL) {
   printk("%s %d oops, gd->dev is NULL\n", __func__, __LINE__);
   return 0;
  }

  dev->size = sdreset(&dev->tssdcore);
  if (dev->size)
    set_capacity(dev->gd, (long long)dev->size * 512 / KERNEL_SECTOR_SIZE);
  return 0;
}

static void tssdcard_sdcommit(unsigned long ldev)
{
  struct tssdcard_dev *dev = (struct tssdcard_dev *) ldev;
  char buf[512];

   //printk("%s %d, dev = 0x%08lX\n", __func__, __LINE__, (unsigned long)dev);

  if (dev == NULL) {
   printk("%s %d oops\n", __func__, __LINE__);
   return;
  }

  //printk("%s %d, LUN: %d\n", __func__, __LINE__, dev->tssdcore.sd_lun);

  if (down_interruptible(&sem)) return; // -ERESTARTSYS;
  sdread(&dev->tssdcore, dev->parksect, buf, 1);
  up(&sem);

}

static int tssdcard_shutdownhook(struct notifier_block *NotifierBlock,
    unsigned long Event, void *Buffer)
{
  int i;
  if (!(Event == SYS_RESTART || Event == SYS_HALT || Event == SYS_POWER_OFF))
    return NOTIFY_DONE;
  for (i = 0; i< ndevices ; i++)
    tssdcard_sdcommit((unsigned long)(devices+i));
  return NOTIFY_OK;
}

static struct notifier_block shutdownhook_notifier = { tssdcard_shutdownhook, NULL, 0 };



/*
 * Thread code.
 */
static int tssdcard_thread(void *data)
{
  struct tssdcard_dev *dev = data;
  struct bio *bio;

  current->flags |= PF_NOFREEZE;
  //set_user_nice(current, -20); /* Highest priority */
  //set_user_nice(current, 19); /* Lower priority */

  while (!kthread_should_stop()) {
    wait_event_interruptible(dev->event, dev->bio ||
         (reqmode != RM_NOQUEUE && blk_peek_request(dev->queue)!=NULL) ||
         kthread_should_stop());
    if (!dev->queue && !dev->bio)
      continue;

    //if (down_trylock(&sem))
    if (down_interruptible(&sem))
      continue;
    atomic_inc(&busy);
    if (atomic_read(&busy) > 1)
      panic("recursive make_request!\n");

    if (reqmode == RM_NOQUEUE) {
      bio = tssdcard_get_bio(dev);
      if(bio) tssdcard_handle_bio(dev, bio);
    } else if (reqmode == RM_FULL)
      tssdcard_handle_fullreq(dev);
    else if (reqmode == RM_SIMPLE)
      tssdcard_handle_simplereq(dev);

    atomic_dec(&busy);
    up(&sem);
  }
  return 0;
}



/*
 * Open and close.
 */
//static int tssdcard_open(struct inode *inode, struct file *filp)
static int tssdcard_open(struct block_device *bdev, fmode_t mode)
{
  struct tssdcard_dev *dev = bdev->bd_disk->private_data;

  if (!atomic_read(&dev->users))
    check_disk_change(bdev);
  atomic_inc(&dev->users);
  if (threnable && dev->thread == NULL && atomic_read(&dev->users)) {
    dev->thread = kthread_create(tssdcard_thread, dev, dev->devname);
    if (IS_ERR(dev->thread))
      dev->thread = NULL;
    else
      wake_up_process(dev->thread);
  }

  return 0;
}


/*
 * The device operations structure.
 */
static struct block_device_operations tssdcard_ops = {
  .owner    = THIS_MODULE,
  .open        = tssdcard_open,
  .release     = tssdcard_release,
  .media_changed  = tssdcard_media_changed,
  .revalidate_disk   = tssdcard_revalidate,
  .ioctl    = tssdcard_ioctl
};


static void setup_device(struct tssdcard_dev *dev, int which)
{
   unsigned int scratchreg0;
   unsigned long conreg;

   memset (dev, 0, sizeof (struct tssdcard_dev));

  //IO Remapping (use the same virtual address for all LUNs)

   if (! ts_host.base) {
      size_t mem_size = resource_size(ts_host.res);
      ts_host.mem_res = request_mem_region(ts_host.res->start, mem_size, DRIVER_NAME);
      if (! ts_host.mem_res) {
         printk("Failed to request memory region\n");
         return;
      }

      ts_host.base = ioremap(ts_host.mem_res->start, mem_size);

      if (IS_ERR(ts_host.base)) {
         printk("Could not map resource\n");
         return;
      }
   }
   dev->tssdcore.sd_regstart = (unsigned int)ts_host.base;
   dev->tssdcore.sd_lun = which;

  //verify previous init
  conreg = (unsigned long)0;
  scratchreg0 = 0;
  if (scratchreg0) {
    dev->tssdcore.sdboot_token = scratchreg0;
   // my_outl(0, 0);
  }

  //dev and tssdcore struct initialization
   dev->tssdcore.os_arg = dev;
   dev->tssdcore.os_delay = tssdcard_delay;
   dev->tssdcore.os_dmastream = NULL;
   dev->tssdcore.os_dmaprep = NULL;
   dev->tssdcore.os_irqwait = NULL;
   dev->tssdcore.sd_writeparking = 1;
   dev->irq = 0;
   dev->irq_wait = 0;
   dev->dmach = dmach;
   dev->dmaregs = 0;
   dev->parksect = 0;

   atomic_set(&dev->users,0);

   //SD Card size and Reset
   dev->size = sdreset(&dev->tssdcore);
   if (dev->size == 0) {
      printk("%s: no card found at LUN %d.\n", DRIVER_NAME, which);
      return;
   }

  dev->gd = alloc_disk(TSSDCARD_MINORS);
  if (! dev->gd) {
    printk (KERN_NOTICE "alloc_disk failure\n");
    return;
  }
  dev->devname = kmalloc(32, GFP_KERNEL);
  strcpy(dev->devname, DRIVER_NAME);
  snprintf(dev->gd->disk_name, 32, strcat(dev->devname,"%c"), which+'a');
  strcpy(dev->devname,dev->gd->disk_name);

   spin_lock_init(&dev->lock);

   if (threnable)
      init_waitqueue_head(&dev->event);

   dev->bio = dev->biotail = NULL;

  //The I/O queue, depending on whether we are using our own make_request function or not.
  switch (reqmode) {
    case RM_NOQUEUE:
      dev->queue = blk_alloc_queue(GFP_KERNEL);
      if (dev->queue == NULL) {
         printk("Can't allocate queue in %s\n", __func__);
         return;
      }
      blk_queue_make_request(dev->queue, tssdcard_make_request);
      break;

    case RM_SIMPLE:
      dev->queue = blk_init_queue(tssdcard_request, &dev->lock);
      if (dev->queue == NULL) return;
      break;

    case RM_FULL:
      dev->queue = blk_init_queue(tssdcard_full_request, &dev->lock);

      if (elevator_init(dev->queue, "noop"))
         printk("Error: Could not set elevator to noop in %s\n", __func__);

      if (dev->queue == NULL) return;
      break;

    default:
      printk(KERN_NOTICE "Bad request mode %d, using simple\n", reqmode);
      dev->queue = blk_init_queue(tssdcard_request, &dev->lock);
      if (dev->queue == NULL) return;
      break;
  }

  blk_queue_logical_block_size(dev->queue, 512);
  dev->queue->queuedata = dev;

  dev->write_activity = 0;

  //And the gendisk structure.
  dev->gd->major = tssdcard_major;
  dev->gd->first_minor = which*TSSDCARD_MINORS;
  dev->gd->flags = GENHD_FL_REMOVABLE;
  dev->gd->fops = &tssdcard_ops;
  dev->gd->queue = dev->queue;
  dev->gd->private_data = dev;
  set_capacity(dev->gd, (long long)dev->size * 512 / KERNEL_SECTOR_SIZE);
  add_disk(dev->gd);

  register_reboot_notifier(&shutdownhook_notifier);

}


static int tssdcard_probe(struct platform_device *pdev)
{
   int i, ret = 0;
   const struct of_device_id *of_id;
   struct device_node *np;
   struct resource *res;
   struct pci_dev *pcidev;

   printk("TS-7800-V2 SD card driver\n");

   of_id = of_match_device(tssdcard_of_match, &pdev->dev);

   if (! of_id)
      return -EINVAL;

   /* The pcie must be enabled before we can access the fpga registers! */

   pcidev = pci_get_device(0x1204, 0x0001, NULL);
   if (!pcidev) {
      printk("Cannot find FPGA at PCI 1204:0001\n");
      return -EINVAL;
   }

   if (pci_enable_device (pcidev)) {
      printk("Cannot enable FPGA at PCI 1204:0001\n");
      return -EINVAL;
   }

   np = pdev->dev.of_node;

   if (of_property_read_u32(np, "tssdcard,ndevices", &ndevices) < 0) {
      pr_info("Can't read property 'tssdcard,ndevices' in device-tree; assuming 2\n");
      ndevices = 2;
   }

   tssdcard_major = register_blkdev(tssdcard_major, DRIVER_NAME);
   if (tssdcard_major <= 0) {
      printk("%s: unable to get major number\n", DRIVER_NAME);
      return -EBUSY;
   }

   devices = kmalloc(ndevices*sizeof (struct tssdcard_dev), GFP_KERNEL);
   if (devices == NULL)
      goto out_unregister;

   sema_init(&sem, 1);
   atomic_set(&busy,0);

   init_waitqueue_head(&qwait);

   if ((res=platform_get_resource(pdev, IORESOURCE_MEM, 0)) == NULL) {
      printk("Can't get IORESOURCE_MEM\n");
      goto out_free;
   }

   ts_host.pdev = pdev;
   ts_host.res = res;
   ts_host.phys_base = res->start;
   ts_host.irq = 0;

   for (i=0; i < ndevices; i++)
      setup_device(devices + i, i);

   platform_set_drvdata(pdev, &ts_host);

   return 0;

out_free:
   kfree(devices);

out_unregister:
   unregister_blkdev(tssdcard_major, DRIVER_NAME);

   return ret;

}

static int tssdcard_remove(struct platform_device *pdev)
{
   struct tssdcard_dev *dev;
   int i;

   if(dbgenable) printk(KERN_NOTICE "%s %d\n", __func__, __LINE__);

   for (i=0; i<ndevices; i++) {
      if(dbgenable) printk(KERN_NOTICE "dev[%d] ...\n", i);

      dev = devices + i;
      if (dev->size == 0) continue;

      if (dev->gd) {
         if(dbgenable) printk(KERN_NOTICE "put_disk() %s\n", dev->gd->disk_name);
         put_disk(dev->gd);

   }

   if (dev->devname)
      kfree(dev->devname);

    if (dev->queue) {
      if (reqmode == RM_NOQUEUE) {
         blk_cleanup_queue(dev->queue);
         blk_put_queue(dev->queue);
         }
      else {
        blk_cleanup_queue(dev->queue);
        }
    }

    if (dev->thread != NULL) {
      kthread_stop(dev->thread);
      dev->thread = NULL;
    }


    if (i == 0) {
      iounmap((unsigned long *) ts_host.base);
      ts_host.base = NULL;
      release_resource(ts_host.mem_res);
    }
  }

  unregister_blkdev(tssdcard_major, DRIVER_NAME);
  kfree(devices);
  devices = NULL;
  return 0;
}

static struct platform_driver tssdcard_driver = {
   .probe      = tssdcard_probe,
   .remove     = tssdcard_remove,
   .driver     = {
      .name    = DRIVER_NAME,
      .of_match_table   = tssdcard_of_match,
   }
};

module_platform_driver(tssdcard_driver);

MODULE_DESCRIPTION("TS-7800-V2 SDHC Driver");
MODULE_AUTHOR("Ian Coughlan, Technologic Systems");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tssdcard");