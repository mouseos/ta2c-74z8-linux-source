#EXTRA CFLAGS
ccflags-y += -Werror

#EXTRA_CFLAGS += -DDEBUG

# Baytrail board

snd-byt-cr-dpcm-rt5640-objs := byt_cr_dpcm_rt5640.o
obj-$(CONFIG_SND_SOC_INTEL_MRD7) += snd-byt-cr-dpcm-rt5640.o  byt_cr_board_configs.o

snd-byt-cr-dpcm-rt5651-objs := byt_cr_dpcm_rt5651.o
obj-$(CONFIG_SND_SOC_INTEL_ANCHOR8) += snd-byt-cr-dpcm-rt5651.o  byt_cr_board_configs.o

# Cherrytrail board
snd-cht-dpcm-rt5672-objs := cht_bl_dpcm_rt5672.o
snd-cht-cr-aic3100-objs := cht_cr_aic3100.o
snd-cht-cr-dpcm-rt5645-objs := cht-cr-dpcm-rt5645.o
snd-cht-cr-dpcm-rt5651-objs := cht_cr_dpcm_rt5651.o
snd-cht-cr-es8316-objs := cht_cr_es8316.o
#obj-$(CONFIG_SND_SOC_INTEL_CHT_T) += snd-cht-dpcm-rt5672.o snd-cht-cr-aic3100.o
obj-$(CONFIG_SND_SOC_INTEL_CHT_T) += snd-cht-cr-dpcm-rt5645.o byt_cr_board_configs.o
obj-$(CONFIG_SND_SOC_ES8316) += snd-cht-cr-es8316.o byt_cr_board_configs.o
obj-$(CONFIG_SND_SOC_RT5651) += snd-cht-cr-dpcm-rt5651.o  byt_cr_board_configs.o
