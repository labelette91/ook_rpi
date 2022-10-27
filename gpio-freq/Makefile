ifneq (${KERNELRELEASE},)

	obj-m  = gpio-freq.o
else

	MODULE_DIR := $(shell pwd)
	KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build

#	KERNEL_DIR = /usr/local/src/linux-rpi-3.6.11
#	ARCH       = arm
#	CROSS_COMPILE = /usr/local/cross/rpi/bin/arm-linux-

	CFLAGS := -Wall -g

all: modules

modules:
	${MAKE} -C ${KERNEL_DIR} M=${MODULE_DIR}  modules
#	./installPulseDriver.sh stop
#	./installPulseDriver.sh start
	sudo rmmod gpio-freq.ko
	sudo insmod /home/pi/rfrpi_src/gpio-freq/gpio-freq.ko gpios=16,22,23,17,24
	sudo chown pi /dev/gpiofreq*
	lsmod | grep gpio_freq

clean:
	rm -f *.o *.ko *.mod.c .*.o .*.ko .*.mod.c .*.cmd *~
	rm -f Module.symvers Module.markers modules.order
	rm -rf .tmp_versions
	
unload:
	sudo rmmod gpio-freq.ko
		

endif
