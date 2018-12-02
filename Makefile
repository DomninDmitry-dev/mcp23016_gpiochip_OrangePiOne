KDIR = $(HOME)/CreateImageOP/cache/sources/linux-mainline/linux-4.14.y
//KDIR = /lib/modules/$(shell uname -r)/build
ARCH = arm
CCFLAGS = -C
COMPILER = arm-linux-gnueabihf-
PWD = $(shell pwd)
TARGET_MOD = mcp23016
TARGET_PROG = test

obj-m   := $(TARGET_MOD).o 

all: prog
	$(MAKE) $(CCFLAGS) $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(COMPILER) modules

prog: $(TARGET_PROG).cpp
	arm-unknown-linux-gnueabihf-g++ $(TARGET_PROG).cpp -o $(TARGET_PROG)
	
clean: 
	@rm -f *.o .*.cmd .*.flags *.mod.c *.order 
	@rm -f .*.*.cmd *~ *.*~ TODO.*
	@rm -fR .tmp* 
	@rm -rf .tmp_versions
	@rm *.ko *.symvers