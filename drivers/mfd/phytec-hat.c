/*
 * PHYTEC RPi HAT Overlay Loader
 *
 * Copyright (C) 2018 PHYTEC America, LLC
 *
 * Based upon bone_capemgr.c
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
 */

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/string.h>
#include <linux/pinctrl/consumer.h>

/* delay to scan on boot until rootfs appears */
static int boot_scan_period = 1000;

struct phytec_hat_data {
	struct i2c_client	*client;
	u16			pid;		/* product ID */
	u16			pver;		/* product version */
	u8			vslen;		/* vendor string length (bytes) */
	u8			pslen;		/* product string length (bytes) */
	char			*vstr;		/* ASCII vendor string */
	char			*pstr;		/* ASCII product string */
	s8			index;		/* known hat index */
	int			overlay;	/* DT overlay id */
}; /* sizeof(phytec_hat_data) == 23 bytes */

struct rpi_hat_data {
#define RPI_HAT_HEADER_SIZE		12		/* bytes */
#define RPI_HAT_HEADER_SIG		0x69502d52	/* "R-Pi" in big-endian ASCII */
#define RPI_HAT_VENDOR_VSTR_OFFSET	42		/* bytes */
	/* EEPROM header */
	u32		signature;
	u8		version;
	u8		reserved;
	u16		numatoms;
	u32		eeplen;
	/* Vendor info atom info (not currently used) */
	u8		vendoratom[8];
	/* Vendor info atom */
	u8		uuid[16];
	u16		pid;
	u16		pver;
	u8		vslen;
	u8		pslen;
}; /* sizeof(rpi_hat_data) == 42 */

typedef struct
{
	u16		pid;		/* product ID */
	u16		pver;		/* product version */
	const char	*dtb;		/* ASCII DT overlay filename */
	const char	*pstr;		/* ASCII product name string */
	const char	*pinctrl;	/* name of pinctrl to load */
} phytec_hat_known_hat;

static const phytec_hat_known_hat phytec_hat_known_hats[] = {
	{	/* Raspberry Pi - Sense HAT */
		0x0001, 0x0001,
		"imx7-peb-d-sense.dtb",
		"Sense HAT",
		"sense_hat",
	},
};

static const int phytec_hat_known_hats_num =
	sizeof(phytec_hat_known_hats)/sizeof(phytec_hat_known_hat);

static int phytec_hat_lookup(struct phytec_hat_data *phytec_hat)
{
	int i;
	size_t pstr_len;

	pstr_len = strlen(phytec_hat->pstr);

	for (i = 0; i < phytec_hat_known_hats_num; i++) {
		if (phytec_hat_known_hats[i].pid == phytec_hat->pid &&
			phytec_hat_known_hats[i].pver == phytec_hat->pver &&
			!strncmp(phytec_hat->pstr,
				phytec_hat_known_hats[i].pstr, pstr_len))
					return i;
	}

	/* didn't find a match */
	return -EINVAL;
}

static int phytec_hat_i2c_read(struct i2c_client *client, u8 *reg, int count,
				u8 *buf)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= reg,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= count,
			.buf	= buf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		dev_err(&client->dev, "%s error %d\n", __func__, ret);

	return ret;
}

static int phytec_hat_read_eeprom(struct phytec_hat_data *phytec_hat,
		struct rpi_hat_data *rpi_hat)
{
	struct i2c_client *client;
	u16 reg;
	int ret;

	client = phytec_hat->client;

	/* hat eeprom addr 0 */
	reg = 0;
	ret = phytec_hat_i2c_read(client, (u8 *)&reg,
		sizeof(struct rpi_hat_data), (u8 *)rpi_hat);
	if (ret < 0)
		return ret;

	return 0;
}

static int phytec_hat_read_strings(struct phytec_hat_data *phytec_hat)
{
	struct i2c_client *client;
	u16 reg;
	int ret;

	client = phytec_hat->client;

	/* hat eeprom addr at which VSTR starts */
	reg = RPI_HAT_VENDOR_VSTR_OFFSET;
	reg = ((0xFF & reg) << 8) | ((0xFF00 & reg) >> 8);
	ret = phytec_hat_i2c_read(client, (u8 *)&reg, phytec_hat->vslen,
		(u8 *)phytec_hat->vstr);
	if (ret < 0)
		return ret;

	/* hat eeprom addr at which VSTR starts + vslen */
	reg = RPI_HAT_VENDOR_VSTR_OFFSET + phytec_hat->vslen;
	reg = ((0xFF & reg) << 8) | ((0xFF00 & reg) >> 8);
	ret = phytec_hat_i2c_read(client, (u8 *)&reg, phytec_hat->pslen,
		(u8 *)phytec_hat->pstr);
	if (ret < 0)
		return ret;

	/* null terminate read strings */
	phytec_hat->vstr[phytec_hat->vslen] = '\0';
	phytec_hat->pstr[phytec_hat->pslen] = '\0';

	return 0;
}

static int phytec_hat_apply_pinctrl(struct phytec_hat_data *phytec_hat)
{
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_state;
	int ret;

	pinctrl = devm_pinctrl_get(&phytec_hat->client->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&phytec_hat->client->dev,
			"%s: unable to get pinctrl!\n", __func__);
		return -EINVAL;
	}

	ret = 0;
	pinctrl_state = pinctrl_lookup_state(pinctrl,
		phytec_hat_known_hats[phytec_hat->index].pinctrl);
	if (!IS_ERR(pinctrl_state))
		ret = pinctrl_select_state(pinctrl, pinctrl_state);

	devm_pinctrl_put(pinctrl);
	pinctrl = NULL;

	return ret;
}

static int phytec_hat_load_overlay(void *data)
{
	const char *dtb;
	const struct firmware *fw;
	struct i2c_client *client;
	struct device_node *overlay;
	struct phytec_hat_data *phytec_hat;
	int ret;

	phytec_hat = data;
	client = phytec_hat->client;
	dtb = phytec_hat_known_hats[phytec_hat->index].dtb;

	/* wait until rootfs is up */
	for (;;) {
		ret = (system_state == SYSTEM_BOOTING);
		if (!ret)
			break;

		msleep(boot_scan_period);

		if (signal_pending(current)) {
			dev_err(&client->dev, "%s: signal pending!\n",
				__func__);
			ret = -ERESTARTSYS;
			goto err;
		}
	}

	/* load devicetree overlay */
	ret = request_firmware_direct(&fw, dtb, &client->dev);
	if (ret)
		goto err;

	/* unflatten overlay */
	ret = (int)of_fdt_unflatten_tree((unsigned long *)fw->data, NULL,
		&overlay);
	if (!overlay || !ret) {
		dev_err(&client->dev, "failed to unflatten %s\n", dtb);
		ret = -EINVAL;
		goto err_release_fw;
	}

	/* mark overlay as detached */
	of_node_set_flag(overlay, OF_DETACHED);

	/* perform resolution on overlay */
	ret = of_resolve_phandles(overlay);
	if (ret < 0) {
		dev_err(&client->dev, "failed to resolve device tree.\n");
		goto err_release_fw;
	}

	/* apply overlay */
	phytec_hat->overlay = of_overlay_create(overlay);
	if (phytec_hat->overlay < 0) {
		dev_err(&client->dev, "failed to create overlay.\n");
		ret = phytec_hat->overlay;
		phytec_hat->overlay = 0;
		goto err_release_fw;
	}

	/* no longer need fw */
	release_firmware(fw);
	fw = NULL;

	dev_info(&client->dev, "overlay loaded:\t%s\n", dtb);
	return 0;

err_release_fw:
	release_firmware(fw);
	fw = NULL;

err:
	dev_err(&client->dev, "failed to load %s!\n", dtb);
	return ret;
}

static int phytec_hat_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct task_struct *kthread;
	struct phytec_hat_data *phytec_hat;
	int ret;
	struct rpi_hat_data *rpi_hat;

	/* allocate space for persistent structs */
	rpi_hat = devm_kzalloc(&client->dev, sizeof(struct rpi_hat_data),
		GFP_KERNEL);
	if (!rpi_hat) {
		dev_err(&client->dev, "%s cannot allocate memory\n", __func__);
		ret = -ENOMEM;
		goto err;
	}

	phytec_hat = devm_kzalloc(&client->dev, sizeof(struct phytec_hat_data),
		GFP_KERNEL);
	if (!phytec_hat) {
		dev_err(&client->dev, "%s cannot allocate memory\n", __func__);
		ret = -ENOMEM;
		goto err_free_rpi_hat;
	}

	/* store i2c_client */
	phytec_hat->client = client;

	/* read rpi hat eeprom data and check it against known hats */
	ret = phytec_hat_read_eeprom(phytec_hat, rpi_hat);
	if (ret < 0) {
		dev_err(&client->dev, "%s unable to read EEPROM!\n", __func__);
		goto err_free_phytec_hat;
	}

	/* check eeprom header */
	if (rpi_hat->signature != RPI_HAT_HEADER_SIG) {
		dev_err(&client->dev, "%s EEPROM header isn't valid!\n",
			__func__);
		ret = -ENODATA;
		goto err_free_phytec_hat;
	}

	/* allocate memory for vendor and product string */
	phytec_hat->vstr = devm_kzalloc(&client->dev, rpi_hat->vslen + 1,
		GFP_KERNEL);
	if (!phytec_hat->vstr) {
		dev_err(&client->dev, "%s cannot allocate memory\n", __func__);
		ret = -ENOMEM;
		goto err_free_phytec_hat;
	}

	phytec_hat->pstr = devm_kzalloc(&client->dev, rpi_hat->pslen + 1,
		GFP_KERNEL);
	if (!phytec_hat->pstr) {
		dev_err(&client->dev, "%s cannot allocate memory\n", __func__);
		ret = -ENOMEM;
		goto err_free_vstr;
	}

	/* set phytec hat data from rpi hat data */
	phytec_hat->pid = rpi_hat->pid;
	phytec_hat->pver = rpi_hat->pver;
	phytec_hat->vslen = rpi_hat->vslen;
	phytec_hat->pslen = rpi_hat->pslen;

	dev_dbg(&client->dev, "%s: phytec_hat->pid is: %x\n", __func__,
		phytec_hat->pid);
	dev_dbg(&client->dev, "%s: phytec_hat->pver is: %x\n", __func__,
		phytec_hat->pver);

	/* read vendor and product strings */
	ret = phytec_hat_read_strings(phytec_hat);
	if (ret < 0)
		goto err_free_pstr;

	dev_dbg(&client->dev, "%s: phytec_hat->vstr is: %s\n", __func__,
		phytec_hat->vstr);
	dev_dbg(&client->dev, "%s: phytec_hat->pstr is: %s\n", __func__,
		phytec_hat->pstr);

	/* check pid, pver, and pstr against known hats and store index */
	phytec_hat->index = phytec_hat_lookup(phytec_hat);
	if (phytec_hat->index < 0) {
		dev_err(&client->dev, "%s read EEPROM vendor " \
			"information doesn't match known HATs!\n",
			__func__);
		goto err_free_pstr;
	}

	dev_dbg(&client->dev, "%s: found HAT at index %i\n", __func__,
		phytec_hat->index);

	/* no longer need rpi hat data */
	devm_kfree(&client->dev, rpi_hat);
	rpi_hat = NULL;

	/* set link to phytec hat data in i2c_client */
	i2c_set_clientdata(client, phytec_hat);

	/* apply pinctrl for HAT */
	if (phytec_hat_known_hats[phytec_hat->index].pinctrl) {
		ret = phytec_hat_apply_pinctrl(phytec_hat);
		if (ret) {
			dev_err(&client->dev, "unable to apply pinctrl!\n");
			goto err_free_pstr;
		}
	}

	/* spawn load overlay kthread */
	if (phytec_hat_known_hats[phytec_hat->index].dtb) {
		kthread = kthread_run(phytec_hat_load_overlay, phytec_hat,
			"phytec-hat-load-overlay");

		if (IS_ERR(kthread)) {
			dev_err(&client->dev,
				"%s failed to spawn load overlay kthread!\n",
				__func__);
			kthread = NULL;
		}
	}

	dev_info(&client->dev, "HAT Vendor:\t\t%s\n", phytec_hat->vstr);
	dev_info(&client->dev, "HAT Product:\t\t%s\n", phytec_hat->pstr);

	return 0;

err_free_pstr:
	devm_kfree(&client->dev, phytec_hat->pstr);
	phytec_hat->pstr = NULL;

err_free_vstr:
	devm_kfree(&client->dev, phytec_hat->vstr);
	phytec_hat->vstr = NULL;

err_free_phytec_hat:
	devm_kfree(&client->dev, phytec_hat);
	phytec_hat = NULL;

err_free_rpi_hat:
	devm_kfree(&client->dev, rpi_hat);
	rpi_hat = NULL;

err:
	dev_err(&client->dev, "failed to initialize device.\n");
	i2c_set_clientdata(client, NULL);

	return ret;
}

static int phytec_hat_remove(struct i2c_client *client)
{
	struct phytec_hat_data *phytec_hat;
	int ret;

	phytec_hat = i2c_get_clientdata(client);
	ret = 0;

	if (phytec_hat->overlay)
		ret = of_overlay_destroy(phytec_hat->overlay);
	if (ret)
		dev_err(&client->dev, "failed to destroy overlay!\n");

	devm_kfree(&client->dev, phytec_hat->pstr);
	devm_kfree(&client->dev, phytec_hat->vstr);
	devm_kfree(&client->dev, phytec_hat);
	phytec_hat = NULL;

	i2c_set_clientdata(client, NULL);

	dev_info(&client->dev, "device uninitialized and removed.\n");

	return ret;
}

static const struct i2c_device_id phytec_hat_device_id[] = {
	{ .name = "phytec-hat", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, phytec_hat_device_id);

static const struct of_device_id phytec_hat_of_match[] = {
	{ .compatible = "phytec,phytec-hat", },
	{ }
};
MODULE_DEVICE_TABLE(of, phytec_hat_of_match);

static struct i2c_driver phytec_hat_driver = {
	.driver = {
		.name		= "phytec-hat",
		.owner		= THIS_MODULE,
		.of_match_table	= phytec_hat_of_match,
	},

	.id_table	= phytec_hat_device_id,
	.probe		= phytec_hat_probe,
	.remove		= phytec_hat_remove,
};
module_i2c_driver(phytec_hat_driver);

MODULE_AUTHOR("Matthew McKee <mmckee@phytec.com>");
MODULE_DESCRIPTION("PHYTEC RPi HAT Overlay Loader");
MODULE_LICENSE("GPL");
