/*
 * linux/include/asm-arm/arch-imx21/tll_mx21.h
 *
 * Modified Aug 29, 2006
 * Tarun Tuli <tarun@virtualcogs.com>
 *
 * Originally based on csb535fs.h
 *
 * Copyright (C) 2004 Robert Schwebel, Pengutronix
 *
 * Ron Melvin (ron.melvin@timesys.com)
 * Copyright (C) 2005 TimeSys Corporation 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the tll_mx21implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * ttcl-vcmx
 */

#ifndef __ASM_ARCH_TLL_MX21_H
#define __ASM_ARCH_TLL_MX21_H

/* ------------------------------------------------------------------------ */
/* Memory Map for the tll_mx21 Board                                         */
/* ------------------------------------------------------------------------ */

#define TLL_MX21_FLASH_PHYS		0x10000000
#define TLL_MX21_FLASH_SIZE		(16*1024*1024)

#define CLK32 32768

#define TLL_MX21_ETH_PHYS CS3_BASE_ADDR
#define TLL_MX21_ETH_VIRT IO_ADDRESS(TLL_MX21_ETH_PHYS)
#define TLL_MX21_ETH_SIZE 0x01000000
#define TLL_MX21_ETH_IRQ  IRQ_GPIOF(0)


#endif /* __ASM_ARCH_TLL_MX21_H */
