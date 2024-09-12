// Copyright (c) 2011-2023 Columbia University, System Level Design Group
// SPDX-License-Identifier: Apache-2.0
#include <linux/of_device.h>
#include <linux/mm.h>

#include <asm/io.h>

#include <esp_accelerator.h>
#include <esp.h>

#include "kalman_sysc_catapult.h"

#define DRV_NAME	"kalman_sysc_catapult"

/* <<--regs-->> */
#define KALMAN_MAC_N_REG 0x48
#define KALMAN_MAC_VEC_REG 0x44
#define KALMAN_MAC_LEN_REG 0x40

struct kalman_sysc_catapult_device {
	struct esp_device esp;
};

static struct esp_driver kalman_driver;

static struct of_device_id kalman_device_ids[] = {
	{
		.name = "SLD_KALMAN_SYSC_CATAPULT",
	},
	{
		.name = "eb_069",
	},
	{
		.compatible = "sld,kalman_sysc_catapult",
	},
	{ },
};

static int kalman_devs;

static inline struct kalman_sysc_catapult_device *to_kalman(struct esp_device *esp)
{
	return container_of(esp, struct kalman_sysc_catapult_device, esp);
}

static void kalman_prep_xfer(struct esp_device *esp, void *arg)
{
	struct kalman_sysc_catapult_access *a = arg;

	/* <<--regs-config-->> */
	iowrite32be(a->mac_n, esp->iomem + KALMAN_MAC_N_REG);
	iowrite32be(a->mac_vec, esp->iomem + KALMAN_MAC_VEC_REG);
	iowrite32be(a->mac_len, esp->iomem + KALMAN_MAC_LEN_REG);
	iowrite32be(a->src_offset, esp->iomem + SRC_OFFSET_REG);
	iowrite32be(a->dst_offset, esp->iomem + DST_OFFSET_REG);

}

static bool kalman_xfer_input_ok(struct esp_device *esp, void *arg)
{
	/* struct kalman_sysc_catapult_device *kalman = to_kalman(esp); */
	/* struct kalman_sysc_catapult_access *a = arg; */

	return true;
}

static int kalman_probe(struct platform_device *pdev)
{
	struct kalman_sysc_catapult_device *kalman;
	struct esp_device *esp;
	int rc;

	kalman = kzalloc(sizeof(*kalman), GFP_KERNEL);
	if (kalman == NULL)
		return -ENOMEM;
	esp = &kalman->esp;
	esp->module = THIS_MODULE;
	esp->number = kalman_devs;
	esp->driver = &kalman_driver;
	rc = esp_device_register(esp, pdev);
	if (rc)
		goto err;

	kalman_devs++;
	return 0;
 err:
	kfree(kalman);
	return rc;
}

static int __exit kalman_remove(struct platform_device *pdev)
{
	struct esp_device *esp = platform_get_drvdata(pdev);
	struct kalman_sysc_catapult_device *kalman = to_kalman(esp);

	esp_device_unregister(esp);
	kfree(kalman);
	return 0;
}

static struct esp_driver kalman_driver = {
	.plat = {
		.probe		= kalman_probe,
		.remove		= kalman_remove,
		.driver		= {
			.name = DRV_NAME,
			.owner = THIS_MODULE,
			.of_match_table = kalman_device_ids,
		},
	},
	.xfer_input_ok	= kalman_xfer_input_ok,
	.prep_xfer	= kalman_prep_xfer,
	.ioctl_cm	= KALMAN_SYSC_CATAPULT_IOC_ACCESS,
	.arg_size	= sizeof(struct kalman_sysc_catapult_access),
};

static int __init kalman_init(void)
{
	return esp_driver_register(&kalman_driver);
}

static void __exit kalman_exit(void)
{
	esp_driver_unregister(&kalman_driver);
}

module_init(kalman_init)
module_exit(kalman_exit)

MODULE_DEVICE_TABLE(of, kalman_device_ids);

MODULE_AUTHOR("Emilio G. Cota <cota@braap.org>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("kalman_sysc_catapult driver");
