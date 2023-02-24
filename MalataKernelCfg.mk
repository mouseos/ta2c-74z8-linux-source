#==============================================================
# Author: xmtdf@malata.com
#==============================================================
ifeq ($(strip $(FEATURE_MEDION_F1018)),yes)
CFLAGS_KERNEL := $(CFLAGS_KERNEL) -DFEATURE_MEDION_F1018
endif

ifeq ($(strip $(FEATURE_SET_PLATFORM_BATT_CHG_PROF_IN_KERNEL)),yes)
CFLAGS_KERNEL := $(CFLAGS_KERNEL) -DFEATURE_SET_PLATFORM_BATT_CHG_PROF_IN_KERNEL
endif

ifneq ($(FEATURE_BATTERY_MODEL),)
CFLAGS_KERNEL := $(CFLAGS_KERNEL) -DFEATURE_BATTERY_MODEL_$(FEATURE_BATTERY_MODEL)
endif

ifeq ($(strip $(FEATURE_HAS_USB3D0_STD_A_CONNECTOR)),yes)
CFLAGS_KERNEL := $(CFLAGS_KERNEL) -DFEATURE_HAS_USB3D0_STD_A_CONNECTOR
endif

ifeq ($(strip $(FEATURE_F1006C)),yes)
CFLAGS_KERNEL := $(CFLAGS_KERNEL) -DFEATURE_F1006C
endif

ifeq ($(strip $(FEATURE_DOCK_USB_INPUT_CURRENT_2A)),yes)
CFLAGS_KERNEL := $(CFLAGS_KERNEL) -DFEATURE_DOCK_USB_INPUT_CURRENT_2A
endif
