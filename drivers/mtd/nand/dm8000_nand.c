/*
 *  drivers/mtd/nand/dm8000.c
 *
 *  Copyright (C) 2000 Steven J. Hill (sjhill@realitydiluted.com)
 *
 *  Modified for Dreambox DM8000 by Felix Domke <tmbinc@elitedvb.net>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Overview:
 *   This is a device driver for the NAND flash device found on the
 *   DM8000 board.
 *
 */

#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/io.h>

static struct mtd_info *dm8000_mtd;
/*
 * Define partitions for flash device
 */
static const struct mtd_partition partition_info[] = {
	{
		.name	= "complete",
		.offset	= 0,
		.size	= 256*1024*1024
	}, {
		.name	= "loader",
		.offset	= 0,
		.size	= 1024*1024
	}, {
		.name	= "boot partition",
		.offset	= 1024*1024,
		.size	= 3*1024*1024
	}, {
		.name	= "root partition",
		.offset	= 4*1024*1024,
		.size	= 60*1024*1024
	}, {
		.name	= "home partition",
		.offset = 64*1024*1024,
		.size	= 64*1024*1024
	}, {
		.name	= "unused partition",
		.offset = 128*1024*1024,
		.size	= 120*1024*1024
	}, {
		.name	= "preset partition",
		.offset = 248*1024*1024,
		.size	= 8*1024*1024
	}
};
#define NUM_PARTITIONS 7

static int dm8000_nand_dev_ready(struct mtd_info *mtd);

static void dm8000_nand_cmd_ctrl(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	if (ctrl & NAND_CTRL_CHANGE && !(ctrl & NAND_NCE))
		writeb(0, (void __iomem *)0xBF030003);

	if (cmd == NAND_CMD_NONE) {
#ifdef CONFIG_SMP
		struct nand_chip *this = (struct nand_chip *)(dm8000_mtd+1);
		int cnt=0;
		while (cnt < this->chip_delay && !dm8000_nand_dev_ready(mtd)) {
			++cnt;
			udelay(1);
		}
#endif
		return;
	}

	if (ctrl & NAND_CLE)
		writeb(cmd, (void __iomem *)0xBF030000);
	else
		writeb(cmd, (void __iomem *)0xBF030002);
}

static void dm8000_nand_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	unsigned int i;

	for (i = 0; i < (len & ~0x03); i += 4)
		*(unsigned long *)&buf[i] = readl(chip->IO_ADDR_R);

	readsb(chip->IO_ADDR_R, &buf[i], len & 0x03);
}

#define SGPIO_OFFSET 10
#define GPIO_NAND_BUSY    (90 + SGPIO_OFFSET)
#define GET_BIT(x, n) (!!((x) & (1<<(n))))

static int gpio_get(int gpio)
{
	static volatile unsigned long * const mmio_gpio = (volatile unsigned long*) 0xb0400700;

	return GET_BIT(mmio_gpio[0x64 / 4], gpio - 96);
}

static int dm8000_nand_dev_ready(struct mtd_info *mtd)
{
	return gpio_get(GPIO_NAND_BUSY);
}

/*
 * Main initialization routine
 */
static int __init dm8000_init(void)
{
	struct nand_chip *this;
	unsigned int probe;
	unsigned int i;
	int ret;
	long j;

	/* Allocate memory for MTD device structure and private data */
	dm8000_mtd = kzalloc(sizeof(struct mtd_info) + sizeof(struct nand_chip),
			     GFP_KERNEL);
	if (!dm8000_mtd) {
		printk(KERN_ERR "Unable to allocate DM8000 NAND MTD device structure.\n");
		return -ENOMEM;
	}

	/* Get pointer to private data */
	this = (struct nand_chip *)&dm8000_mtd[1];

	writeb(NAND_CMD_READID, (void __iomem *)0xBF030000);
	writeb(0, (void __iomem *)0xBF030002);
	probe = readl((void __iomem *)0xBF030004);
	writeb(0, (void __iomem *)0xBF030003); // term

	j = jiffies;
	for (i = 0; i < 1000 * 1000 / 4; i++)
		readl((void __iomem *)0xBF030004);

	printk(KERN_INFO "%ld kb/s\n", 1000 * HZ / (jiffies - j));

	printk(KERN_INFO " - NAND PROBE: %02x %02x %02x %02x\n",
	       (probe >>  0) & 0xff,
	       (probe >>  8) & 0xff,
	       (probe >> 16) & 0xff,
	       (probe >> 24) & 0xff);

	/* Link the private data with the MTD structure */
	dm8000_mtd->priv = this;
	dm8000_mtd->owner = THIS_MODULE;

	/* Set address of NAND IO lines */
	this->IO_ADDR_R = (void __iomem *)0xBF030004;
	this->IO_ADDR_W = (void __iomem *)0xBF030004;
	/* Set address of hardware control function */
	this->cmd_ctrl = dm8000_nand_cmd_ctrl;
	this->read_buf = dm8000_nand_read_buf;
	/* command delay time (in us) */
	this->chip_delay = 15;
	/* return the status of the Ready/Busy line */
	this->dev_ready = dm8000_nand_dev_ready;
	this->ecc.mode = NAND_ECC_SOFT;

	/* Scan to find existence of the device */
	if (nand_scan(dm8000_mtd, 1)) {
		ret = -ENXIO;
		goto err_kfree;
	}

	/* Register the partitions */
	ret = add_mtd_partitions(dm8000_mtd, partition_info, NUM_PARTITIONS);
	if (ret < 0)
		goto err_nand_release;

	/* Return happy */
	return 0;

err_nand_release:
	nand_release(dm8000_mtd);
err_kfree:
	kfree(dm8000_mtd);
	return ret;
}

module_init(dm8000_init);

/*
 * Clean up routine
 */
static void __exit dm8000_cleanup(void)
{
	/* Release resource, unregister partitions */
	nand_release(dm8000_mtd);

	/* Free the MTD device structure */
	kfree(dm8000_mtd);
}

module_exit(dm8000_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Felix Domke <tmbinc@elitedvb.net>");
MODULE_DESCRIPTION("Dream-Multimedia DM8000 NAND flash board glue");
