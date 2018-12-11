/*
 * Copyright (C) 2015-2016 Mentor Graphics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/watchdog.h>
#include <linux/reboot.h>

#include "watchdog_pretimeout.h"

/**
 * pretimeout_ctrlaltdel - Panic on watchdog pretimeout event
 * @wdd - watchdog_device
 *
 * Panic, watchdog has not been fed till pretimeout event.
 */
static void pretimeout_ctrlaltdel(struct watchdog_device *wdd)
{
	ctrl_alt_del();
}

static struct watchdog_governor watchdog_gov_ctrlaltdel = {
	.name		= "ctrlaltdel",
	.pretimeout	= pretimeout_ctrlaltdel,
};

static int __init watchdog_gov_ctrlaltdel_register(void)
{
	return watchdog_register_governor(&watchdog_gov_ctrlaltdel);
}

static void __exit watchdog_gov_ctrlaltdel_unregister(void)
{
	watchdog_unregister_governor(&watchdog_gov_ctrlaltdel);
}
module_init(watchdog_gov_ctrlaltdel_register);
module_exit(watchdog_gov_ctrlaltdel_unregister);

MODULE_AUTHOR("Leon Woestenberg <leon@sidebranch.com>");
MODULE_DESCRIPTION("CTRL-ALT-DEL watchdog pretimeout governor");
MODULE_LICENSE("GPL");
