# Makefile for Hotplugs
# Don't allow optimizations unless specified.
cflags-y	+= -Os
CFLAGS_alucard_hotplug.o += -w

obj-$(CONFIG_ALUCARD_HOTPLUG)           += alucard_hotplug.o
obj-$(CONFIG_ASMP)           	        += autosmp.o
obj-$(CONFIG_DYNAMIC_HOTPLUG)           += dynamic_hotplug.o
obj-$(CONFIG_INTELLI_PLUG)              += intelli_plug.o
