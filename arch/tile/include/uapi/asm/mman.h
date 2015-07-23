/*
 * Copyright 2010 Tilera Corporation. All Rights Reserved.
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation, version 2.
 *
 *   This program is distributed in the hope that it will be useful, but
 *   WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, GOOD TITLE or
 *   NON INFRINGEMENT.  See the GNU General Public License for
 *   more details.
 */

#ifndef _ASM_TILE_MMAN_H
#define _ASM_TILE_MMAN_H

#include <asm-generic/mman-common.h>
#include <arch/chip.h>

/* Standard Linux flags */

#define MAP_POPULATE	0x0040		/* populate (prefault) pagetables */
#define MAP_NONBLOCK	0x0080		/* do not block on IO */
#define MAP_GROWSDOWN	0x0100		/* stack-like segment */
#define MAP_STACK	MAP_GROWSDOWN	/* provide convenience alias */
#define MAP_LOCKED	0x0200		/* pages are locked */
#define MAP_NORESERVE	0x0400		/* don't check for reservations */
#define MAP_DENYWRITE	0x0800		/* ETXTBSY */
#define MAP_EXECUTABLE	0x1000		/* mark it as an executable */
#define MAP_HUGETLB	0x4000		/* create a huge page mapping */
#define MAP_LOCKONFAULT	0x100000	/* Lock pages after they are faulted in, do not prefault */


/*
 * Flags for mlockall
 */
#define MCL_CURRENT	1		/* lock all current mappings */
#define MCL_FUTURE	2		/* lock all future mappings */
#define MCL_ONFAULT	4		/* lock all pages that are faulted in */


/*
 * Flags for mlock
 */
#define MLOCK_LOCKED	0x01		/* Lock and populate the specified range */
#define MLOCK_ONFAULT	0x02		/* Lock pages in range after they are faulted in, do not prefault */


#endif /* _ASM_TILE_MMAN_H */
