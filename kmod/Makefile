# SPDX-License-Identifier: GPL-2.0

obj-$(CONFIG_VIDEO_AP1302) += ap1302.o

AP1302_DRV_TAG:= $(shell git -C $(src) describe  --tag --long --always || echo Unknown)
CFLAGS_ap1302.o:=-DAP1302_DRV_TAG='"$(AP1302_DRV_TAG)"'

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(CURDIR) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(CURDIR) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symversa

