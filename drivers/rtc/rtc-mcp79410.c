/*
 * MICROCHIP MCP79410 rtc class driver
 *
 * Copyright (C) 2011, Goodwin+
 * Author: Sharapov Sergey <sas@goodwin.ru>
 *
 * Based on rtc-isl1208
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <linux/of.h>

#define DRV_VERSION "0.2"

#define MCP79410_RTC_REG	   0x00
#define MCP79410_CTRL_REG	  0x07
#define MCP79410_CALIB_REG	 0x08
#define MCP79410_RTC_SECTION_LEN   0x07

// offsets into CCR area
#define MCP79410_REG_SC  0
#define MCP79410_REG_MN  1
#define MCP79410_REG_HR  2
#define MCP79410_REG_DW  3
#define MCP79410_REG_MD  4
#define MCP79410_REG_MO  5
#define MCP79410_REG_YR  6

#define MCP79410_REG_RUN	  (1<<7) // run rtc
#define MCP79410_REG_SQOUT	(1<<6) // square output enable
#define MCP79410_REG_HR_MIL       (1<<6) // 24h/12h mode
#define MCP79410_REG_HR_PM	(1<<5) // AM/PM bit in 12h mode

static struct i2c_driver mcp79410_driver;


static int
mcp79410_i2c_read_regs(struct i2c_client *client, u8 start_addr, u8 *pdest,
		       unsigned len)
{
	int ret;
	unsigned char addr = start_addr;
	struct i2c_msg msgs[] = {
		{
			.addr  = client->addr,
			.flags  = 0,
			.len  = 1,
			.buf  = &addr,
		},
		{
			.addr  = client->addr,
			.flags  = I2C_M_RD,
			.len  = len,
			.buf  = pdest,
		},
	};
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != 2) {
		dev_err(&client->dev, "read error (%d)\n", ret);
		return (ret < 0) ? ret : -EIO;
	}
	return 0;
}


static int
mcp79410_i2c_set_regs(struct i2c_client *client, u8 reg, const u8 *psrc,
		      unsigned len)
{
	int ret;
	u8 i2c_buf[MCP79410_RTC_SECTION_LEN+1];
	i2c_buf[0] = reg;
	if (len > sizeof(i2c_buf) - 1)
		len = sizeof(i2c_buf)-1;

	memcpy(i2c_buf+1, psrc, len);
	ret = i2c_master_send(client, i2c_buf,  len+1);
	if (ret != (len+1)) {
		dev_err(&client->dev, "write error (%d)\n", ret);
		return (ret < 0) ? ret : -EIO;
	}
	return 0;
}

static int
mcp79410_read_time(struct i2c_client *client, struct rtc_time *tm)
{
	int res;
	u8 regs[MCP79410_RTC_SECTION_LEN];
	res = mcp79410_i2c_read_regs(client, MCP79410_RTC_REG, regs, sizeof(regs));
	if (res < 0) {
		dev_err(&client->dev, "reading RTC section failed\n");
		return res;
	}
	tm->tm_sec = bcd2bin(regs[MCP79410_REG_SC]&0x7F);
	tm->tm_min = bcd2bin(regs[MCP79410_REG_MN])&0x7F;
	// HR field has a more complex interpretation
	if ((regs[MCP79410_REG_HR] & MCP79410_REG_HR_MIL)==0) {
		// 24h format
		tm->tm_hour = bcd2bin(regs[MCP79410_REG_HR]&0x3F);
	} else {
		// 12h format
		tm->tm_hour = bcd2bin(regs[MCP79410_REG_HR]&0x1F);
		if (regs[MCP79410_REG_HR] & MCP79410_REG_HR_PM) // PM flag set
			tm->tm_hour += 12;
	}
	tm->tm_mday = bcd2bin(regs[MCP79410_REG_MD]&0x3F);
	tm->tm_mon  = bcd2bin(regs[MCP79410_REG_MO]&0x1F) - 1;
	tm->tm_year = bcd2bin(regs[MCP79410_REG_YR])+100;
	tm->tm_wday = bcd2bin(regs[MCP79410_REG_DW]&0x07);
	return rtc_valid_tm(tm);
}

static int
mcp79410_set_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	int ret;
	unsigned char buf[MCP79410_RTC_SECTION_LEN];

	dev_dbg(&client->dev,  "%s: mday=%d, mon=%d, year=%d, wday=%d\n",
		__func__, tm->tm_mday, tm->tm_mon, tm->tm_year, tm->tm_wday);
	dev_dbg(&client->dev,  "%s: secs=%d, mins=%d, hours=%d\n",
		__func__, tm->tm_sec, tm->tm_min, tm->tm_hour);

	buf[MCP79410_REG_SC] = (bin2bcd(tm->tm_sec) & 0x7F) | MCP79410_REG_RUN;
	buf[MCP79410_REG_MN] = bin2bcd(tm->tm_min) & 0x7F;
	buf[MCP79410_REG_HR] = bin2bcd(tm->tm_hour) & 0x3F;
	buf[MCP79410_REG_MD] = bin2bcd(tm->tm_mday) & 0x3F;
	buf[MCP79410_REG_MO] = bin2bcd(tm->tm_mon + 1) & 0x1F;
	buf[MCP79410_REG_YR] = bin2bcd(tm->tm_year % 100);
	/* 0x8 below enables battery supply */
	buf[MCP79410_REG_DW] = (tm->tm_wday & 0x07) | 0x8;

	ret = mcp79410_i2c_set_regs(client, MCP79410_RTC_REG, buf, sizeof(buf));
	if (ret < 0)
		return ret;
	buf[0] = 0xC0;
	ret = mcp79410_i2c_set_regs(client, MCP79410_CTRL_REG, buf, 1);
	if (ret < 0)
		return ret;
	return 0;
}

static int mcp79410_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	return mcp79410_read_time(to_i2c_client(dev), tm);
}

static int mcp79410_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	return mcp79410_set_datetime(to_i2c_client(dev), tm);
}

static int mcp79410_rtc_ioctl(struct device *dev, unsigned int cmd,
			      unsigned long arg)
{
	int res;
	u8 reg;
	void __user *uarg = (void __user *) arg;
	struct rtc_pll_info info;


	//  dev_info(dev, "ioctl: cmd=%08x, arg=%08lx\n", cmd, arg);
	switch (cmd) {
	case RTC_PLL_GET:
		res = mcp79410_i2c_read_regs(to_i2c_client(dev),
		        MCP79410_CALIB_REG, &reg, sizeof(reg));
		if (res < 0) {
			dev_err(dev, "reading CALIBR reg failed\n");
			break;
		}
		/* pll_posmult and pll_negmult should be 2/60 for 32kHz clock,
		since adjusments are made once in a minute, and number of
		periods substracted or added is 2*pll_value. Since it's not
		possible to pass fractional numbers, clock is multiplied
		by (60/2) instead.
		If pll_value > 0, then clock is sped up,
		if pll_value < 0, then clock is slowed down.
		*/
		info.pll_ctrl = 0;
		info.pll_value = (reg & 0x80) ? (reg & 0x7F) : (-((int) reg));
		info.pll_max = 127;
		info.pll_min = -127;
		info.pll_posmult = 1;
		info.pll_negmult = 1;
		info.pll_clock = 32768 * 60 / 2;
		if (copy_to_user(uarg, &info, sizeof(info)))
			res = -EFAULT;
		break;

	case RTC_PLL_SET:
		if (copy_from_user(&info, uarg, sizeof(info))) {
			res = -EFAULT;
			break;
		}

		if (info.pll_value < -0x7f || 0x7f < info.pll_value) {
			res = -EINVAL;
			break;
		}

		if (info.pll_value < 0)
			reg = -info.pll_value;
		else
			reg = 0x80 | info.pll_value;

		res = mcp79410_i2c_set_regs(to_i2c_client(dev),
		        MCP79410_CALIB_REG, &reg, sizeof(reg));
		if (res < 0) {
			dev_err(dev, "writing CALIBR reg failed\n");
			break;
		}
		break;

	default:
		res = -ENOIOCTLCMD;
	}
	return res;
}

static const struct rtc_class_ops mcp79410_rtc_ops = {
	.read_time  = mcp79410_rtc_read_time,
	.set_time   = mcp79410_rtc_set_time,
	.ioctl      = mcp79410_rtc_ioctl,
};

static DEVICE_ATTR(id, S_IRUGO, NULL, NULL);


static int
mcp79410_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;
	struct rtc_device *rtc = NULL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;
	dev_info(&client->dev, "chip found, driver version " DRV_VERSION "\n");
	rtc = rtc_device_register(mcp79410_driver.driver.name,
				  &client->dev, &mcp79410_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);
	i2c_set_clientdata(client, rtc);
	// Register sysfs hooks
	rc = device_create_file(&client->dev, &dev_attr_id);
	if (rc < 0)
		goto exit_unregister;
	return 0;

exit_unregister:
	rtc_device_unregister(rtc);
	return rc;
}

static int
mcp79410_remove(struct i2c_client *client)
{
	struct rtc_device *rtc = i2c_get_clientdata(client);
	rtc_device_unregister(rtc);
	device_remove_file(&client->dev, &dev_attr_id);
	return 0;
}

static const struct i2c_device_id mcp79410_id[] = {
	{ "mcp79410", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, mcp79410_id);

#ifdef CONFIG_OF
static const struct of_device_id mcp79410_of_match[] = {
	{ .compatible = "microchip,mcp79410" },
	{}
};
MODULE_DEVICE_TABLE(of, pcf8563_of_match);
#endif

static struct i2c_driver mcp79410_driver = {
	.driver = {
		.name = "rtc-mcp79410",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(mcp79410_of_match),
	},
	.probe = mcp79410_probe,
	.remove = mcp79410_remove,
	.id_table = mcp79410_id,
};

static int __init mcp79410_init(void)
{
	return i2c_add_driver(&mcp79410_driver);
}

static void __exit mcp79410_exit(void)
{
	i2c_del_driver(&mcp79410_driver);
}

MODULE_AUTHOR("Sharapov Sergey <sas@goodwin.ru>");
MODULE_DESCRIPTION("Microchip mcp79410 driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

module_init(mcp79410_init);
module_exit(mcp79410_exit);
