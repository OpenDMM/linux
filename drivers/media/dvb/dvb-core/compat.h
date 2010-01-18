#ifndef _COMPAT_H
#define _COMPAT_H

#include <linux/i2c-id.h>
#include <linux/version.h>
#include <linux/utsname.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)
# define class_device_create(a, b, c, d, e, f, g, h) class_simple_device_add(a, c, d, e, f, g, h)
# define class_device_destroy(a, b...) class_simple_device_remove(b)
# define class_create class_simple_create
# define class_destroy class_simple_destroy
# define class class_simple
# define try_to_freeze() do { if (current->flags & PF_FREEZE) refrigerator(PF_FREEZE); } while(0)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
# define class_device_create(a, b, c, d, e, f, g, h) class_device_create(a, c, d, e, f, g, h)
#endif

#endif
