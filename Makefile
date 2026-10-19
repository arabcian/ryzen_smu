MOD 				:= ryzen_smu
VERSION				:= 0.1.9
TARGET				:= $(shell uname -r)
DKMS_ROOT_PATH			:= /usr/src/$(MOD)-$(VERSION)

KERNEL_MODULES			:= /lib/modules/$(TARGET)

# Manual install: `make && sudo make install`. DESTDIR is honoured for staging
# (packaging / Gentoo ebuild image dir). kbuild runs depmod itself.
DESTDIR				?=
INSTALL_MOD_DIR			:= extra

ifneq ("","$(wildcard /usr/src/linux-headers-$(TARGET)/*)")
	KERNEL_BUILD		:= /usr/src/linux-headers-$(TARGET)
else
ifneq ("","$(wildcard /usr/src/kernels/$(TARGET)/*)")
	KERNEL_BUILD		:= /usr/src/kernels/$(TARGET)
else
	KERNEL_BUILD		:= $(KERNEL_MODULES)/build
endif
endif

obj-m				:= $(MOD).o
$(MOD)-objs		 	:= drv.o smu.o

.PHONY: all modules clean install uninstall dkms-install dkms-uninstall insmod checkmod

all: modules

debug:
	@$(MAKE) -C $(KERNEL_BUILD) M=$(CURDIR) ccflags-y+=-DDEBUG modules

modules:
	@$(MAKE) -C $(KERNEL_BUILD) M=$(CURDIR) modules

clean:
	@$(MAKE) -C $(KERNEL_BUILD) M=$(CURDIR) clean
	rm -rf *.o

# kbuild's modules_install handles signing (CONFIG_MODULE_SIG_ALL) and
# compression (CONFIG_MODULE_COMPRESS_*) the same way in-tree modules get it.
install: modules
	@$(MAKE) -C $(KERNEL_BUILD) M=$(CURDIR) \
		INSTALL_MOD_PATH=$(DESTDIR) INSTALL_MOD_DIR=$(INSTALL_MOD_DIR) \
		modules_install

uninstall:
	-modprobe -r $(MOD) 2>/dev/null
	rm -f $(DESTDIR)$(KERNEL_MODULES)/$(INSTALL_MOD_DIR)/$(MOD).ko \
	      $(DESTDIR)$(KERNEL_MODULES)/$(INSTALL_MOD_DIR)/$(MOD).ko.*
	depmod -a $(if $(DESTDIR),-b $(DESTDIR)) $(TARGET)

dkms-install:
	mkdir -p $(DKMS_ROOT_PATH)
	cp $(CURDIR)/dkms.conf $(DKMS_ROOT_PATH)
	cp $(CURDIR)/Makefile $(DKMS_ROOT_PATH)
	cp $(CURDIR)/*.c $(DKMS_ROOT_PATH)
	cp $(CURDIR)/*.h $(DKMS_ROOT_PATH)

	sed -e "s/@CFLGS@/${MCFLAGS}/" \
		-e "s/@VERSION@/$(VERSION)/" \
		-i $(DKMS_ROOT_PATH)/dkms.conf

	dkms add ryzen_smu/$(VERSION)
	dkms build ryzen_smu/$(VERSION)
	dkms install ryzen_smu/$(VERSION)

dkms-uninstall:
	dkms remove ryzen_smu/$(VERSION) --all
	rm -rf $(DKMS_ROOT_PATH)

insmod:
	sudo rmmod $(MOD).ko; true
	sudo insmod $(MOD).ko

checkmod:
	lsmod | grep $(MOD)
	cat /proc/modules | grep $(MOD)
	cat /sys/kernel/$(MOD)_drv/drv_version
	modinfo $(MOD)
