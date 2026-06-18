DRIVER ?= lme2510_dtmb

obj-m += $(DRIVER).o

# 内核源码路径（按实际系统调整）
KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a
	@echo "安装完成，加载命令："
	@echo "  sudo modprobe lme2510_dtmb debug=1"

# 调试加载
load:
	sudo rmmod $(DRIVER) 2>/dev/null || true
	sudo insmod $(DRIVER).ko debug=1
	@echo "加载完成，查看日志："
	@echo "  sudo dmesg | tail -30"

unload:
	sudo rmmod $(DRIVER) 2>/dev/null || true
