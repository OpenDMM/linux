/*
 * Copyright (C) 2004  Maciej W. Rozycki
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#ifndef _ASM_COMPILER_H
#define _ASM_COMPILER_H

#include <asm/types.h>

#if __GNUC__ > 3 || (__GNUC__ == 3 && __GNUC_MINOR__ >= 4)
#define GCC_REG_ACCUM "$0"
#else
#define GCC_REG_ACCUM "accum"
#endif

#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 4)
#define GCC_NO_H_CONSTRAINT
#ifdef CONFIG_64BIT
typedef unsigned int uintx_t __attribute__((mode(TI)));
#else
typedef u64 uintx_t;
#endif
#endif

#endif /* _ASM_COMPILER_H */
