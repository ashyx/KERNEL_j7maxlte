#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

obj-$(CONFIG_MTK_ION) += ion_drv.o ion_mm_heap.o ion_fb_heap.o ion_dma_reserved_heap.o
ccflags-$(CONFIG_MTK_ION)  += -I$(srctree)/drivers/staging/android/ion #fro kernel-3.10 ion
ccflags-$(CONFIG_MTK_M4U) += -I$(srctree)/drivers/misc/mediatek/m4u/$(MTK_PLATFORM)/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mmp/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/arch/arm/

ifeq ($(CONFIG_MTK_ENG_BUILD), y)
ccflags-y += -DION_RUNTIME_DEBUGGER=1
ccflags-y += -DION_HISTORY_RECORD
obj-$(CONFIG_MTK_ION) += ion_history.o
else
ccflags-y += -DION_RUNTIME_DEBUGGER=0
endif

ccflags-y += -I$(srctree)/drivers/misc/mediatek/secmem/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/secmem/$(MTK_PLATFORM)/

ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/trustzone

ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
	MTK_SEC_SERVICE=y
endif
ifeq ($(CONFIG_MTK_IN_HOUSE_TEE_SUPPORT),y)
	MTK_SEC_SERVICE=y
endif
ifeq ($(CONFIG_BLOWFISH_TEE_SUPPORT),y)
	MTK_SEC_SERVICE=y
endif

ifeq ($(MTK_SEC_SERVICE),y)
	ccflags-y += -DMTK_SEC_SERVICE
	obj-$(CONFIG_MTK_ION) += ion_sec_heap.o
endif

