/*
* ***************************************************************************
* Copyright (C) 2016 Marvell International Ltd.
* ***************************************************************************
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
* ***************************************************************************
*/

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/usb/usb_phy_generic.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/usb/hcd.h>

#define USB2_VBUS_DET_SEL_OFF		12
#define USB2_VBUS_DET_SEL_MASK		0x3
#define USB2_VBUS_DET_SEL_AVALID	2

#define USB_HOST_MODE_ACT_OFF		0
#define USB_HOST_MODE_DEACT_OFF		1
#define USB_DEVICE_MODE_ACT_OFF		4
#define USB_DEVICE_MODE_DEACT_OFF	5

#define USB_ID_DEVICE			1
#define USB_ID_HOST			0

enum port_status {
	USB_PORT_IDLE,
	USB_HOST_ATTACHED,
	USB_DEVICE_ATTACHED,
};

struct a3700_otg_regs {
	u32 usb_ctrl_mode;	/* USB32 Control Mode */
	u32 usb_id_sts;		/* USB32 ID Status */
	u32 usb_isr;		/* Interrupt Status */
	u32 usb_ier;		/* Interrupt Enable */
	u32 usb_clk_ctrl;	/* USB32 Clock Control */
	u32 rsv0;		/* Reserved */
	u32 rsv1;		/* Reserved */
	u32 rsv2;		/* Reserved */
	u32 usb_otg_phy_ctrl;	/* USB2 Host and Device OTG PHY Control */
};

struct a3700_otg {
	struct usb_phy phy;
	struct a3700_otg_regs *otg_regs;

	struct device *dev;
	int irq;

	struct regulator *vcc;

	struct delayed_work work;
	struct workqueue_struct *qwork;

	enum port_status old_state;
	enum port_status port_state;
	bool host_started;
};

#define set_bit(nr, val) (val |= 1 << (nr))
#define clear_bit(nr, val) (val &= ~(1 << (nr)))
#define test_bit(nr, val) (val & (1 << (nr)))

#define VBUS_IRQ_FLAGS \
	(IRQF_SHARED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | \
		IRQF_ONESHOT)

static void a3700_otg_start_host(struct a3700_otg *mvotg, int on)
{
	struct usb_hcd *hcd;
	struct usb_otg *otg = mvotg->phy.otg;

	if ((!otg->host) || ((!mvotg->host_started) && (!on)))
		return;

	dev_dbg(mvotg->dev, "%s, %s\n", __func__, on ? "on":"off");

	hcd = bus_to_hcd(otg->host);

	if (on) {
		mvotg->host_started = true;
		usb_add_hcd(hcd, hcd->irq, IRQF_SHARED);
		usb_add_hcd(hcd->shared_hcd, hcd->irq, IRQF_SHARED);
		device_wakeup_enable(hcd->self.controller);
	} else {
		if (hcd->state == HC_STATE_RUNNING) {
			usb_remove_hcd(hcd->shared_hcd);
			usb_remove_hcd(hcd);
		} else {
			usb_remove_hcd(hcd);
			usb_remove_hcd(hcd->shared_hcd);
		}
		mvotg->host_started = false;
	}

	if (!IS_ERR(mvotg->vcc)) {
		if (on) {
			if (regulator_enable(mvotg->vcc))
				dev_err(mvotg->dev, "Failed to enable power\n");
		} else {
			if (regulator_disable(mvotg->vcc))
				dev_err(mvotg->dev, "Failed to disable power\n");
		}
	}
}

static void a3700_otg_start_periphrals(struct a3700_otg *mvotg, int on)
{
	dev_dbg(mvotg->dev, "%s, %s\n", __func__, on ? "on":"off");
}

void a3700_otg_enable_irq(struct a3700_otg *mvotg)
{
	u32 reg_val;

	reg_val = readl(&mvotg->otg_regs->usb_ier);
	clear_bit(USB_HOST_MODE_ACT_OFF, reg_val);
	clear_bit(USB_HOST_MODE_DEACT_OFF, reg_val);
	clear_bit(USB_DEVICE_MODE_ACT_OFF, reg_val);
	clear_bit(USB_DEVICE_MODE_DEACT_OFF, reg_val);
	writel(reg_val, &mvotg->otg_regs->usb_ier);
};
void a3700_otg_disable_irq(struct a3700_otg *mvotg)
{
	u32 reg_val;

	reg_val = readl(&mvotg->otg_regs->usb_ier);
	set_bit(USB_HOST_MODE_ACT_OFF, reg_val);
	set_bit(USB_HOST_MODE_DEACT_OFF, reg_val);
	set_bit(USB_DEVICE_MODE_ACT_OFF, reg_val);
	set_bit(USB_DEVICE_MODE_DEACT_OFF, reg_val);
	writel(reg_val, &mvotg->otg_regs->usb_ier);
};

static void a3700_otg_work(struct work_struct *work)
{
	struct a3700_otg *mvotg;

	mvotg = container_of(to_delayed_work(work), struct a3700_otg, work);

	switch (mvotg->old_state) {
	case USB_PORT_IDLE:
		if (mvotg->port_state == USB_HOST_ATTACHED) {
			dev_dbg(mvotg->dev, "moving to host mode\n");
			a3700_otg_start_host(mvotg, 1);
		} else if (mvotg->port_state == USB_DEVICE_ATTACHED) {
			dev_dbg(mvotg->dev, "moving to device mode\n");
			a3700_otg_start_periphrals(mvotg, 1);
		}
		break;
	case USB_HOST_ATTACHED:
		if (mvotg->port_state == USB_PORT_IDLE) {
			dev_dbg(mvotg->dev, "moving to idle mode\n");
			a3700_otg_start_host(mvotg, 0);
		}
		break;
	case USB_DEVICE_ATTACHED:
		if (mvotg->port_state == USB_PORT_IDLE) {
			dev_dbg(mvotg->dev, "moving to idle mode\n");
			a3700_otg_start_periphrals(mvotg, 0);
		}
		break;
	default:
		break;
	}
}

static void a3700_otg_run_state_machine(struct a3700_otg *mvotg,
				     unsigned long delay)
{
	dev_dbg(mvotg->dev, "transceiver is updated\n");
	if (!mvotg->qwork)
		return;

	queue_delayed_work(mvotg->qwork, &mvotg->work, delay);
}

static irqreturn_t a3700_usb_id_isr(int irq, void *data)
{
	u32 reg_val;
	u32 usb_id;
	enum port_status port_state_original;
	struct a3700_otg *mvotg = data;

	a3700_otg_disable_irq(mvotg);

	reg_val = readl(&mvotg->otg_regs->usb_isr);
	usb_id = test_bit(USB_HOST_MODE_ACT_OFF, readl(&mvotg->otg_regs->usb_id_sts));
	port_state_original = mvotg->port_state;
	dev_dbg(mvotg->dev, "receive usb-id interrupt!, status: 0x%x, usb_id: %d\n", reg_val, usb_id);

	switch (mvotg->port_state) {
	case USB_PORT_IDLE:
		dev_dbg(mvotg->dev, "current state USB_PORT_IDLE!\n");
		if (test_bit(USB_HOST_MODE_ACT_OFF, reg_val) && (usb_id == USB_ID_HOST)) {
			mvotg->port_state = USB_HOST_ATTACHED;
		} else if (test_bit(USB_DEVICE_MODE_ACT_OFF, reg_val)
			&& !test_bit(USB_DEVICE_MODE_DEACT_OFF, reg_val)
			&& usb_id == USB_ID_DEVICE) {
			mvotg->port_state = USB_DEVICE_ATTACHED;
		}
		break;
	case USB_HOST_ATTACHED:
		dev_dbg(mvotg->dev, "current state USB_HOST_ATTACHED!\n");
		if (test_bit(USB_HOST_MODE_DEACT_OFF, reg_val))
			mvotg->port_state = USB_PORT_IDLE;
		break;
	case USB_DEVICE_ATTACHED:
		dev_dbg(mvotg->dev, "current state USB_DEVICE_ATTACHED!\n");
		if (test_bit(USB_DEVICE_MODE_DEACT_OFF, reg_val))
			mvotg->port_state = USB_PORT_IDLE;
		break;
	default:
		dev_err(mvotg->dev, "Unknown state found!\n");
		break;
	}

	if (port_state_original != mvotg->port_state) {
		mvotg->old_state = port_state_original;
		a3700_otg_run_state_machine(mvotg, 0);
	}

	/* ack irq */
	writel(reg_val, &mvotg->otg_regs->usb_isr);

	a3700_otg_enable_irq(mvotg);

	return IRQ_HANDLED;
}

static int a3700_otg_set_peripheral(struct usb_otg *otg, struct usb_gadget *gadget)
{
	if (!otg)
		return -ENODEV;

	if (!gadget) {
		otg->gadget = NULL;
		return -ENODEV;
	}

	otg->gadget = gadget;
	otg->state = OTG_STATE_B_IDLE;
	return 0;
}

static int a3700_otg_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	if (!otg)
		return -ENODEV;

	if (!host) {
		otg->host = NULL;
		return -ENODEV;
	}

	otg->host = host;
	return 0;
}

int a3700_otg_phy_create_phy(struct device *dev, struct a3700_otg *mvotg,
		struct usb_phy_generic_platform_data *pdata)
{
	enum usb_phy_type type = USB_PHY_TYPE_USB2;
	bool needs_vcc = false;

	if (dev->of_node) {
		struct device_node *node = dev->of_node;

		needs_vcc = of_property_read_bool(node, "vcc-supply");

		mvotg->otg_regs = of_iomap(node, 0);
		if (IS_ERR(mvotg->otg_regs))
			return -EINVAL;

		mvotg->irq = irq_of_parse_and_map(node, 0);

	} else {
		dev_err(dev, "Can't find otg node\n");
		return -ENODEV;
	}

	mvotg->phy.otg = devm_kzalloc(dev, sizeof(*mvotg->phy.otg),
			GFP_KERNEL);
	if (!mvotg->phy.otg)
		return -ENOMEM;

	mvotg->vcc = devm_regulator_get(dev, "vcc");
	if (IS_ERR(mvotg->vcc)) {
		dev_dbg(dev, "Error getting vcc regulator: %ld\n",
					PTR_ERR(mvotg->vcc));
		if (needs_vcc)
			return -EPROBE_DEFER;
	}

	mvotg->dev		= dev;
	mvotg->phy.dev		= mvotg->dev;
	mvotg->phy.label		= "a3700-otg";
	mvotg->phy.type		= type;

	mvotg->phy.otg->state		= OTG_STATE_UNDEFINED;
	mvotg->phy.otg->usb_phy		= &mvotg->phy;
	mvotg->phy.otg->set_host		= a3700_otg_set_host;
	mvotg->phy.otg->set_peripheral	= a3700_otg_set_peripheral;

	return 0;
}

static int a3700_otg_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct a3700_otg	*mvotg;
	int err;

	mvotg = devm_kzalloc(dev, sizeof(*mvotg), GFP_KERNEL);
	if (!mvotg)
		return -ENOMEM;

	err = a3700_otg_phy_create_phy(dev, mvotg, dev_get_platdata(&pdev->dev));
	if (err)
		return err;

	err = usb_add_phy_dev(&mvotg->phy);
	if (err) {
		dev_err(&pdev->dev, "can't register transceiver, err: %d\n",
			err);
		return err;
	}

	platform_set_drvdata(pdev, mvotg);

	mvotg->port_state = USB_PORT_IDLE;
	mvotg->host_started = false;

	mvotg->qwork = create_singlethread_workqueue("a3700_otg_queue");
	if (!mvotg->qwork) {
		dev_dbg(&pdev->dev, "cannot create workqueue for OTG\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&mvotg->work, a3700_otg_work);

	/* unmask irq */
	a3700_otg_enable_irq(mvotg);

	if (mvotg->irq) {
		ret = request_irq(mvotg->irq, a3700_usb_id_isr, IRQF_TRIGGER_HIGH, "usb-id", mvotg);
		if (ret) {
			dev_err(&pdev->dev, "failed to request gpio interrupt, ret: %x\n", ret);
			return -EFAULT;
		}
	}

	return ret;
}

static int a3700_otg_remove(struct platform_device *pdev)
{
	struct a3700_otg *mvotg = platform_get_drvdata(pdev);

	if (mvotg->qwork) {
		flush_workqueue(mvotg->qwork);
		destroy_workqueue(mvotg->qwork);
	}

	free_irq(mvotg->irq, mvotg);

	usb_remove_phy(&mvotg->phy);

	iounmap(mvotg->otg_regs);
	return 0;
}

static const struct of_device_id a3700_otg_dt_ids[] = {
	{ .compatible = "marvell,armada-3700-otg" },
	{ }
};

MODULE_DEVICE_TABLE(of, a3700_otg_dt_ids);

static struct platform_driver a3700_otg_driver = {
	.probe		= a3700_otg_probe,
	.remove		= a3700_otg_remove,
	.driver		= {
		.name	= "a3700_otg_phy",
		.of_match_table = a3700_otg_dt_ids,
	},
};

static int __init a3700_otg_init(void)
{
	return platform_driver_register(&a3700_otg_driver);
}
subsys_initcall(a3700_otg_init);

static void __exit a3700_otg_exit(void)
{
	platform_driver_unregister(&a3700_otg_driver);
}
module_exit(a3700_otg_exit);

MODULE_ALIAS("platform:a3700_otg_phy");
MODULE_AUTHOR("Terry Zhou <bjzhou@marvell.com>");
MODULE_DESCRIPTION("Marvell Armada3700 otg phy driver");
MODULE_LICENSE("GPL");
