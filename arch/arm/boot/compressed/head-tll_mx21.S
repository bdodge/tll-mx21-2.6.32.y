/* linux/arch/arm/boot/compressed/head-vcmx212.S
 * 
 * Modified Sept 29, 2006 
 * Tarun Tuli <tarun@virtualcogs.com>
 *
 * Originally based on head-csb535.S by Jay Monkman
 * Copyright (C) 2005 Jay Monkman <jtm@lopingdog.com>
 * 
 * Some code borrowed from Nicolas Pitre's 'head-sa1100.S' file. This
 * is merged with head.S by the linker.
 * ttcl-vcmx
 */

#include <linux/linkage.h>
#include <asm/mach-types.h>

#ifndef CONFIG_MACH_TLL_MX21
#error Using head-tll_mx21.S, but system isnt a TLL MX21!
#endif

	.section        ".start", "ax"

__VCMX212_start:
	b	4f
1:	.word	MACH_TYPE_TLL_MX21 /* Machine type */
2:	.word   0x10027000         /* PLL regs phys addr */
3:	.word   0x2e000000         /* Value for PCCR1 - enable GPT clocks */

4:
	ldr	r1, 2b
	ldr	r2, 3b
	str	r2, [r1, #36]    /* Write to PCCR1 */

	ldr	r7, 1b           /* Set machine type */

