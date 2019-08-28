# Makefile for Hotplugs
# Don't allow optimizations unless specified.
cflags-y	+= -Os
CFLAGS_lazyplug.o	 += -finline-functions
CFLAGS_alucard_hotplug.o += -w

obj-$(CONFIG_ALUCARD_HOTPLUG)           += alucard_hotplug.o
obj-$(CONFIG_ASMP)           	        += autosmp.o
obj-$(CONFIG_BLU_PLUG)                  += blu_plug.o
obj-$(CONFIG_BRICKED_HOTPLUG)           += bricked_hotplug.o
obj-$(CONFIG_DYNAMIC_HOTPLUG)           += dynamic_hotplug.o
obj-$(CONFIG_MSM_SLEEPER)		+= msm_sleeper.o
obj-$(CONFIG_LAZYPLUG)                  += lazy_plug.o
obj-$(CONFIG_STATE_HELPER)		+= state_helper.o
obj-$(CONFIG_THUNDER_PLUG)              += thunder_plug.o
