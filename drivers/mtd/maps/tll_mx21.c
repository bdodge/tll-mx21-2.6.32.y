/*
 * Flash memory access on TLL MX21 board 
 * 
 *  Copyright (C) 2004 Metrowerks
 *  Copyright (C) 2004 Motorola Semiconductors SuZhou Ltd
 *  Copyright (C) 2006 Jochen Karrer 
 *
 *  Based on maps/sa1100-flash.c, which is:
 *   (C) 2000 Nicolas Pitre <nico@cam.org>
 *
 *  Based on mx21fs3-flash.c which is:
 *   (C) 2006 Jochen Karrer
 * 
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>


#ifndef CONFIG_ARCH_MX2
#error This is for i.MX2 based boards only
#endif

#define FLASH_BASE (0xc8000000)

/*------------------- TLL MX21 board partitions ------------------------*/

static struct map_info tll_mx21_map = {
	.name =	"TLL MX21 Flash",
	.bankwidth = 2,
	.phys = FLASH_BASE
};

/*
 * Here is the partition information for all flash on the tll_mx21.
 * See include/linux/mtd/partitions.h for definition of the mtd_partition
 * structure.
 */
static struct mtd_partition tll_mx21_partitions[] =
{
	{
		name:		"uMon",
		size:		0x800000,			
		offset:		0x00000000,
		mask_flags:	MTD_WRITEABLE,      /* this makes it read only */
	}, 
	{
		name:		"jffs2",
		size:		MTDPART_SIZ_FULL,		/* Remainder of space for JFFS2 */
		offset:		MTDPART_OFS_APPEND,
	}
};


static struct mtd_info *mymtd;
static struct map_info *mtd_map;

static int __init mx21_mtd_init(void)
{
	struct mtd_partition *parts;
	int nb_parts = 0, ret;
	const char *part_type;
	unsigned long base = -1UL;

	/*
	 * Static partition definition selection
	 */
	part_type = "static";

	if (1) {
		mtd_map       = &tll_mx21_map;
		base          = 0xc8000000;
		parts         = tll_mx21_partitions;
		nb_parts      = ARRAY_SIZE(tll_mx21_partitions);
		mtd_map->size = 0x1000000;     /* 16M */
	} else {
		printk(KERN_NOTICE "MTD error: not TLL i.MX21 board\n");
		return -ENXIO;
	}

	/*
	 * For simple flash devices, use ioremap to map the flash.
	 */
	if (base != (unsigned long) -1) {
		if (!request_mem_region(base, mtd_map->size, "flash"))
			return -EBUSY;

		mtd_map->phys = base;
		mtd_map->virt =  ioremap(base, mtd_map->size);

		ret = -ENOMEM;
		if (!mtd_map->virt)
			goto out_err;

		simple_map_init(mtd_map);
	}

	/*
	 * Now let's probe for the actual flash.  Do it here since
	 * specific machine settings might have been set above.
	 */
	printk(KERN_NOTICE "%s: probing %d-bit flash bus\n", 
	       mtd_map->name, mtd_map->bankwidth*8);
	
	mymtd = do_map_probe("cfi_probe", mtd_map);
	if (!mymtd) {
		mymtd = do_map_probe("jedec_probe", mtd_map);
	}
	if (!mymtd) {
		ret = -ENXIO;
		goto out_err;
	}

	mymtd->owner = THIS_MODULE;

#ifdef CONFIG_MTD_PARTITIONS
	printk(KERN_NOTICE "Using %s partition definition\n", part_type);
	add_mtd_partitions(mymtd, parts, nb_parts);
#else
	printk(KERN_NOTICE "Using no partitions, add device only\n", part_type);
	add_mtd_device(mymtd);
#endif
	return 0;

 out_err:
	if (mtd_map->phys != -1) {
		iounmap((void *)mtd_map->virt);
	}
	return ret;
}

static void __exit mx21_mtd_cleanup(void)
{
	if (mymtd) {
		del_mtd_partitions(mymtd);
		map_destroy(mymtd);
	}
	if (mtd_map && mtd_map->phys != -1) {
		iounmap((void *)mtd_map->virt);
	}
}

module_init(mx21_mtd_init);
module_exit(mx21_mtd_cleanup);

MODULE_DESCRIPTION("i.MX21 CFI map driver");
MODULE_LICENSE("GPL");
