// Copyright (c) 2011-2023 Columbia University, System Level Design Group
// SPDX-License-Identifier: Apache-2.0
#include <linux/of_device.h>
#include <linux/mm.h>

#include <asm/io.h>

#include <esp_accelerator.h>
#include <esp.h>

#include "mac_2_sysc_catapult.h"

#define DRV_NAME	"mac_2_sysc_catapult"

/* <<--regs-->> */
#define MAC_2_MAC_N_REG 0x48
#define MAC_2_MAC_VEC_REG 0x44
#define MAC_2_MAC_LEN_REG 0x40

struct mac_2_sysc_catapult_device {
	struct esp_device esp;
};

static struct esp_driver mac_2_driver;

static struct of_device_id mac_2_device_ids[] = {
	{
		.name = "SLD_MAC_2_SYSC_CATAPULT",
	},
	{
		.name = "eb_057",
	},
	{
		.compatible = "sld,mac_2_sysc_catapult",
	},
	{ },
};

static int mac_2_devs;

static inline struct mac_2_sysc_catapult_device *to_mac_2(struct esp_device *esp)
{
	return container_of(esp, struct mac_2_sysc_catapult_device, esp);
}

static void mac_2_prep_xfer(struct esp_device *esp, void *arg)
{
	struct mac_2_sysc_catapult_access *a = arg;

	/* <<--regs-config-->> */
	iowrite32be(a->mac_n, esp->iomem + MAC_2_MAC_N_REG);
	iowrite32be(a->mac_vec, esp->iomem + MAC_2_MAC_VEC_REG);
	iowrite32be(a->mac_len, esp->iomem + MAC_2_MAC_LEN_REG);
	iowrite32be(a->src_offset, esp->iomem + SRC_OFFSET_REG);
	iowrite32be(a->dst_offset, esp->iomem + DST_OFFSET_REG);

}

static bool mac_2_xfer_input_ok(struct esp_device *esp, void *arg)
{
	/* struct mac_2_sysc_catapult_device *mac_2 = to_mac_2(esp); */
	/* struct mac_2_sysc_catapult_access *a = arg; */

	return true;
}

static int mac_2_probe(struct platform_device *pdev)
{
	struct mac_2_sysc_catapult_device *mac_2;
	struct esp_device *esp;
	int rc;

	mac_2 = kzalloc(sizeof(*mac_2), GFP_KERNEL);
	if (mac_2 == NULL)
		return -ENOMEM;
	esp = &mac_2->esp;
	esp->module = THIS_MODULE;
	esp->number = mac_2_devs;
	esp->driver = &mac_2_driver;
	rc = esp_device_register(esp, pdev);
	if (rc)
		goto err;

	mac_2_devs++;
	return 0;
 err:
	kfree(mac_2);
	return rc;
}

static int __exit mac_2_remove(struct platform_device *pdev)
{
	struct esp_device *esp = platform_get_drvdata(pdev);
	struct mac_2_sysc_catapult_device *mac_2 = to_mac_2(esp);

	esp_device_unregister(esp);
	kfree(mac_2);
	return 0;
}

static struct esp_driver mac_2_driver = {
	.plat = {
		.probe		= mac_2_probe,
		.remove		= mac_2_remove,
		.driver		= {
			.name = DRV_NAME,
			.owner = THIS_MODULE,
			.of_match_table = mac_2_device_ids,
		},
	},
	.xfer_input_ok	= mac_2_xfer_input_ok,
	.prep_xfer	= mac_2_prep_xfer,
	.ioctl_cm	= MAC_2_SYSC_CATAPULT_IOC_ACCESS,
	.arg_size	= sizeof(struct mac_2_sysc_catapult_access),
};

static int __init mac_2_init(void)
{
	return esp_driver_register(&mac_2_driver);
}

static void __exit mac_2_exit(void)
{
	esp_driver_unregister(&mac_2_driver);
}

module_init(mac_2_init)
module_exit(mac_2_exit)

MODULE_DEVICE_TABLE(of, mac_2_device_ids);

MODULE_AUTHOR("Emilio G. Cota <cota@braap.org>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mac_2_sysc_catapult driver");
