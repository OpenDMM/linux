/*
 *
 * BRIEF MODULE DESCRIPTION
 *	Bcm97402 PCI Controller specific pci setup.
 *
 * Copyright 2004-2006 Broadcom Corp.
 *
 * This file was derived from the sample file pci_ops.c
 *
 * Carsten Langgaard, carstenl@mips.com
 * Copyright (C) 1999,2000 MIPS Technologies, Inc.  All rights reserved.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Revision Log
 * who when    what
 * tht  041004 Adapted from sample codes from kernel tree
 */
#include <linux/init.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/ioport.h>

//#include <asm/pci_channel.h>

//extern const unsigned long mips_io_port_base;

static struct resource pci_io_resource = {
	.name = "io pci IO space",
	.start = 0x0,
	.end = 0x0000000 + 0x0060000B,
	.flags = IORESOURCE_IO,
};

static struct resource pci_mem_resource = {
	.name = "ext pci memory space",
	.start = 0xd0000000,
	.end = 0xd0000000 + 0x08000000,
	.flags = IORESOURCE_MEM,
};



extern struct pci_ops bcm7402_pci_ops; 

struct pci_controller bcm7402_controller = {
	.pci_ops 		= &bcm7402_pci_ops, 
	.io_resource 	= &pci_io_resource, 
	.mem_resource   = &pci_mem_resource,
};


static int __init brcm97402_pci_init(void)
{
	register_pci_controller(&bcm7402_controller);
	return 0;
}

arch_initcall(brcm97402_pci_init);




