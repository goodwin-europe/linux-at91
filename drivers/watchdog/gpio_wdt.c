/*
 * Driver for a generic GPIO-triggered watchdog.

 * Copyright (C) 2011, Goodwin+
 * Author: Alexander Morozov <etesial@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/watchdog.h>

struct gpio_wdt_data {
	struct watchdog_device wdt;
	struct kref refcount;

	struct timer_list timer;
	int gpio;
	int gpio_state;
	int timer_interval; /* in jiffies */
	volatile int use_timer;
};

static const struct of_device_id of_gpio_wdt_match[] = {
	{ .compatible = "generic,gpio-wdt", },
	{},
};


static void gpio_wdt_trigger(struct gpio_wdt_data *data)
{
	data->gpio_state = !data->gpio_state;
	gpio_set_value(data->gpio, data->gpio_state);
}


static void gpio_wdt_timer(unsigned long arg)
{
	struct gpio_wdt_data *data = (struct gpio_wdt_data *) arg;

	if (data->use_timer)
		mod_timer(&data->timer, jiffies + data->timer_interval);

	gpio_wdt_trigger(data);
}

static void gpio_wdt_start_timer(struct gpio_wdt_data *data)
{
	/* Access is serialized by the watchdog framework,
	so no additional locking is required. */
	if (!data->use_timer) {
		gpio_wdt_trigger(data);
		data->use_timer = 1;
		mod_timer(&data->timer, jiffies + data->timer_interval);
	}
}

static void gpio_wdt_stop_timer(struct gpio_wdt_data *data)
{
	if (data->use_timer) {
		data->use_timer = 0;
		del_timer_sync(&data->timer);
	}
}


static void gpio_wdt_release_resources(struct kref *r)
{
	struct gpio_wdt_data *data =
		container_of(r, struct gpio_wdt_data, refcount);

	gpio_wdt_stop_timer(data);
	if (data->gpio >= 0)
		gpio_free(data->gpio);

	/* Data allocated by devm_kzalloc() will be freed on device removal */
}

static int gpio_wdt_start(struct watchdog_device *wdt_dev) {
	struct gpio_wdt_data *data = watchdog_get_drvdata(wdt_dev);

	gpio_wdt_stop_timer(data);
	return 0;
}

static int gpio_wdt_stop(struct watchdog_device *wdt_dev) {
	struct gpio_wdt_data *data = watchdog_get_drvdata(wdt_dev);

	gpio_wdt_start_timer(data);
	return 0;
}

static int gpio_wdt_ping(struct watchdog_device *wdt_dev) {
	struct gpio_wdt_data *data = watchdog_get_drvdata(wdt_dev);

	gpio_wdt_trigger(data);
	return 0;
}

static void gpio_wdt_ref(struct watchdog_device *wdt_dev)
{
	struct gpio_wdt_data *data = watchdog_get_drvdata(wdt_dev);

	kref_get(&data->refcount);
}

static void gpio_wdt_unref(struct watchdog_device *wdt_dev)
{
	struct gpio_wdt_data *data = watchdog_get_drvdata(wdt_dev);

	kref_put(&data->refcount, gpio_wdt_release_resources);
}



static const struct watchdog_ops gpio_wdt_ops = {
	.owner = THIS_MODULE,
	.start = gpio_wdt_start,
	.stop = gpio_wdt_stop,
	.ping = gpio_wdt_ping,
	.ref = gpio_wdt_ref,
	.unref = gpio_wdt_unref,
};

static struct watchdog_info gpio_wdt_info = {
	.options = WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE |
	           WATCHDOG_NOWAYOUT_INIT_STATUS,
	.identity = "Generic GPIO Watchdog",
};


static int gpio_wdt_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct gpio_wdt_data *data;
	uint32_t interval;
	int ret;

	dev_dbg(&pdev->dev, "Probe");
	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto err;
	}
	kref_init(&data->refcount);

	data->gpio = of_get_gpio(np, 0);
	if (gpio_cansleep(data->gpio)) {
		dev_err(&pdev->dev, "GPIO controllers that can sleep "
			"are not supported");
		ret = -ENODEV;
		goto err;
	}
	data->gpio_state = gpio_get_value(data->gpio);
	ret = gpio_request_one(data->gpio, GPIOF_DIR_OUT,
			np->name);
	if (ret)
		goto err_g;

	if (of_property_read_u32(np, "ping-sec", &interval)) {
		dev_err(&pdev->dev, "WDT ping interval is not defined");
		ret = -EINVAL;
		goto err_g;
	}
	data->timer_interval = interval * HZ;
	data->use_timer = 0;
	init_timer(&data->timer);
	setup_timer(&data->timer, gpio_wdt_timer, (unsigned long) data);


	if (of_property_read_u32(np, "timeout-sec", &interval)) {
		dev_err(&pdev->dev, "WDT timeout is not defined");
		ret = -EINVAL;
		goto err_g;
	}
	data->wdt.timeout = interval * HZ;
	data->wdt.min_timeout = data->wdt.timeout;
	data->wdt.max_timeout = data->wdt.timeout;
	data->wdt.info = &gpio_wdt_info;
	data->wdt.ops = &gpio_wdt_ops;
	watchdog_set_drvdata(&data->wdt, data);
	platform_set_drvdata(pdev, data);

	/* timer must be started before watchdog framework is initialized
	   to evade race condition in timer_start/stop functions */
	gpio_wdt_start_timer(data);

	ret = watchdog_register_device(&data->wdt);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register watchdog: %d", ret);
		goto err_t;
	}

	return 0;

err_t:
	gpio_wdt_stop_timer(data);
err_g:
	gpio_free(data->gpio);
err:
	return ret;
}

static int gpio_wdt_remove(struct platform_device *pdev)
{
	struct gpio_wdt_data *data = platform_get_drvdata(pdev);

	watchdog_unregister_device(&data->wdt);
	if (!kref_put(&data->refcount, gpio_wdt_release_resources))
		dev_err(&pdev->dev, "WDT was not released");

	return 0;
}

static struct platform_driver gpio_wdt_driver = {
	.probe = gpio_wdt_probe,
	.remove = gpio_wdt_remove,
	.driver		= {
		.name	= "gpio-wdt",
		.owner	= THIS_MODULE,
		.of_match_table = of_gpio_wdt_match,
	},
};

module_platform_driver(gpio_wdt_driver);

MODULE_AUTHOR("Alexander Morozov");
MODULE_DESCRIPTION("Driver for a generic GPIO-triggered watchdog");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:gpio-wdt");
