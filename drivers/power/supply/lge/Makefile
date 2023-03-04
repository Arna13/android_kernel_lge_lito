ifeq ($(CONFIG_LGE_PM_ACTM), y)
obj-y		+= adaptive-charging-thermal.o
endif
obj-y		+= charging-ceiling.o
obj-y		+= protection-battemp.o
obj-y		+= protection-batvolt.o
obj-y		+= unified-bootmode.o
obj-y		+= unified-nodes.o
obj-y		+= unified-sysfs.o
obj-y		+= veneer-psy.o
obj-y		+= veneer-uevent.o
obj-y		+= veneer-voter.o
ifneq ($(CONFIG_LGE_PM_CCD), y)
obj-y		+= charging-time-v2.o
obj-y		+= protection-showcase.o
obj-y		+= protection-usbio.o
endif
ifeq ($(CONFIG_LGE_PM_QNOVO_QNS), y)
obj-y		+= qns_system.o
endif