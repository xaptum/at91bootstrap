CPPFLAGS += -DCONFIG_XAPRC001
ASFLAGS += -DCONFIG_XAPRC001

ifeq ($(CONFIG_XAPRC001_PROV),y)
        CPPFLAGS += -DCONFIG_XAP_INIT_DDR_SELF_REFRESH
        ASFLAGS += -DCONFIG_XAP_INIT_DDR_SELF_REFRESH
endif
