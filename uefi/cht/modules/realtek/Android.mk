#add build switch xmysx@20160130
ifeq (${COMBO_CHIP_VENDOR},rtl)
# build this as an external kernel modules
$(eval $(call build_kernel_module,$(call my-dir)/,rtl8723bs,))
endif # COMBO_CHIP_VENDOR
#add build switch xmysx@20160130 end