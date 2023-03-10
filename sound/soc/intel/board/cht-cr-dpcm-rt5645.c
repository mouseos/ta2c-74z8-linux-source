/*
 *  cht_cr_dpcm_rt5645.c - ASoc DPCM Machine driver
 *  for Intel CherryTrail MID platform
 *
 *  Copyright (C) 2014 Intel Corp
 *  This file is modified from byt_bl_rt5642.c for cherrytrail
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/vlv2_plat_clock.h>
#include <linux/mutex.h>
#include <linux/dmi.h>
#include <asm/platform_cht_audio.h>
#include <asm/intel-mid.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "../../codecs/rt5645/rt5645.h"

#define _ML "RT5645 "
//#define CHT_RT5645_JD_SOC 406 

#define CHT_PLAT_CLK_3_HZ	19200000

#define CHT_INTR_DEBOUNCE               0
#define CHT_HS_DET_DELAY                100
#define CHT_HS_INSERT_DET_DELAY         400
#define CHT_HS_REMOVE_DET_DELAY         500
#define CHT_BUTTON_DET_DELAY            100
#define CHT_HS_DET_POLL_INTRVL          100
#define CHT_BUTTON_EN_DELAY             1500

#define CHT_HS_DET_RETRY_COUNT          6

struct cht_mc_private {
	struct snd_soc_jack jack;
	struct delayed_work hs_detect_work;
	struct delayed_work hs_insert_work;
	struct delayed_work hs_remove_work;
	struct delayed_work hs_button_work;
	struct mutex jack_mlock;
	/* To enable button press interrupts after a delay after HS detection.
	 * This is to avoid spurious button press events during slow
	 * HS insertion
	 */
	struct delayed_work hs_button_en_work;
	int intr_debounce;
	int hs_det_delay;
	int hs_insert_det_delay;
	int hs_remove_det_delay;
	int button_det_delay;
	int button_en_delay;
	int hs_det_poll_intrvl;
	int hs_det_retry;
	bool process_button_events;

};

static int cht_hs_detection(void *);

static struct snd_soc_jack_gpio hs_gpio = {
		.name			= "cht-codec-int",
		.report			= SND_JACK_HEADSET |
					  SND_JACK_HEADPHONE |
					  SND_JACK_BTN_0,
		.debounce_time		= CHT_INTR_DEBOUNCE,
		.jack_status_check	= cht_hs_detection,
};
static inline void cht_force_enable_pin(struct snd_soc_codec *codec,
			 const char *bias_widget, bool enable)
{
	pr_debug("%s %s\n", enable ? "enable" : "disable", bias_widget);
	if (enable)
		snd_soc_dapm_force_enable_pin(&codec->dapm, bias_widget);
	else
		snd_soc_dapm_disable_pin(&codec->dapm, bias_widget);
}

static inline void cht_set_codec_power(struct snd_soc_codec *codec,
								int jack_type)
{
	const char *board_name;
	switch (jack_type) {
	case SND_JACK_HEADSET:
		board_name = dmi_get_system_info(DMI_BOARD_NAME);
		pr_debug("Setting the micbias for %s\n", board_name);
		if (strcmp(board_name, "Cherry Trail FFD") == 0)
			cht_force_enable_pin(codec, "micbias1", true);
		else
			cht_force_enable_pin(codec, "micbias2", true);
		cht_force_enable_pin(codec, "JD Power", true);
		cht_force_enable_pin(codec, "Mic Det Power", true);
		break;
	case SND_JACK_HEADPHONE:
		cht_force_enable_pin(codec, "JD Power", true);
		cht_force_enable_pin(codec, "Mic Det Power", false);
		cht_force_enable_pin(codec, "micbias2", false);
		break;
	case 0:
		cht_force_enable_pin(codec, "JD Power", true);
		cht_force_enable_pin(codec, "Mic Det Power", false);
		cht_force_enable_pin(codec, "micbias2", true); // depend on hardware
	       break;
	default:
		return;
	}
	snd_soc_dapm_sync(&codec->dapm);
}
int cht_mc_check_jd_status(struct snd_soc_codec *codec){
	unsigned int status;
#ifdef CHT_RT5645_JD_SOC
	status = gpio_get_value(CHT_RT5645_JD_SOC);
	pr_info(_ML "%s, jd1 pin status:%x\n", __func__, status);
#else
	status = rt5645_check_jd_status(codec);
#endif
	return  status;
}


/* Identify the jack type as Headset/Headphone/None */
static int cht_check_jack_type(struct snd_soc_jack *jack,
					struct snd_soc_codec *codec)
{
	int status, jack_type = 0;
	const char *board_name;
	struct cht_mc_private *ctx = container_of(jack,
					struct cht_mc_private, jack);

	status = cht_mc_check_jd_status(codec);
	/* jd status low indicates some accessory has been connected */
	if (!status) {
		pr_info("Jack insert intr\n");
		/* Do not process button events until
		accessory is detected as headset*/
		ctx->process_button_events = false;
		cht_set_codec_power(codec, SND_JACK_HEADSET);
		board_name = dmi_get_system_info(DMI_BOARD_NAME);
/*
		if (strcmp(board_name, "Cherry Trail Tablet") == 0) {
			rt5670_supported_hs_type(codec,
					RT5670_HS_RING4_MICBIAS2);
		} else if (strcmp(board_name, "Cherry Trail FFD") == 0) {
			rt5670_supported_hs_type(codec,
					RT5670_HS_RING4_MICBIAS1);
		}
*/
		jack_type = rt5645_headset_detect(codec, true);
		if (jack_type == SND_JACK_HEADSET) {
			ctx->process_button_events = true;
			/* If headset is detected, enable
			button interrupts after a delay */
			schedule_delayed_work(&ctx->hs_button_en_work,
					msecs_to_jiffies(ctx->button_en_delay));
		}
		if (jack_type != SND_JACK_HEADSET)
			cht_set_codec_power(codec, SND_JACK_HEADPHONE);
	} else
		jack_type = 0;

	pr_info("Jack type detected:%d\n", jack_type);

	return jack_type;
}

/* Work function invoked by the Jack Infrastructure.
 * Other delayed works for jack detection/removal/button
 * press are scheduled from this function
 */
static int cht_hs_detection(void *data)
{
	int status, jack_type = 0;
	int ret;
	struct snd_soc_jack_gpio *gpio = &hs_gpio;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	struct cht_mc_private *ctx = container_of(jack,
						struct cht_mc_private, jack);


	pr_info("Enter:%s, before mutex lock\n", __func__);
	mutex_lock(&ctx->jack_mlock);
	/* Initialize jack status with previous status.
	 * Delayed work will confirm the event and send updated status later
	 */
	jack_type = jack->status;
	pr_info("Enter:%s, after mutex lock\n", __func__);

	if (!jack->status) {
		ctx->hs_det_retry = CHT_HS_DET_RETRY_COUNT;
		ret = schedule_delayed_work(&ctx->hs_insert_work,
				msecs_to_jiffies(ctx->hs_insert_det_delay));
		if (!ret)
			pr_info("cht_check_hs_insert_status already queued\n");
		else
			pr_info("%s:Check hs insertion  after %d msec\n",
					__func__, ctx->hs_insert_det_delay);

	} else {
		/* First check for accessory removal; If not removed,
		 * check for button events
		 */
		status = cht_mc_check_jd_status(codec);
		/* jd status high indicates accessory has been disconnected.
		 * However, confirm the removal in the delayed work
		 */
		if (status) {
			/* Do not process button events while we make sure
			 * accessory is disconnected
			 */
			ctx->process_button_events = false;
			ret = schedule_delayed_work(&ctx->hs_remove_work,
				msecs_to_jiffies(ctx->hs_remove_det_delay));
			if (!ret)
				pr_info("remove work already queued\n");
			else
				pr_info("%s:Check hs removal after %d msec\n",
					__func__, ctx->hs_remove_det_delay);
		} else {
			/* Must be button event.
			Confirm the event in delayed work*/
			if (((jack->status & SND_JACK_HEADSET) ==
						SND_JACK_HEADSET) &&
					ctx->process_button_events) {
				ret = schedule_delayed_work(
					&ctx->hs_button_work,
					msecs_to_jiffies(
						ctx->button_det_delay));
				if (!ret)
					pr_info("button_work already queued\n");
				else
					pr_info("%s:check BP/BR after %d msec\n",
					__func__, ctx->button_det_delay);
			}
		}
	}

	mutex_unlock(&ctx->jack_mlock);
	pr_info("Exit:%s\n", __func__);
	return jack_type;
}
/* Checks jack insertion and identifies the jack type.
 * Retries the detection if necessary
 */
static void cht_check_hs_insert_status(struct work_struct *work)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	struct cht_mc_private *ctx = container_of(work,
				struct cht_mc_private, hs_insert_work.work);
	int jack_type = 0;

	mutex_lock(&ctx->jack_mlock);
	pr_info("Enter:%s\n", __func__);

	jack_type = cht_check_jack_type(jack, codec);

	/* Report jack immediately only if jack is headset.
	 * If headphone or no jack was detected, dont report it
	 * until the last HS det try.
	 * This is to avoid reporting any temporary jack removal or
	 * accessory change (eg, HP to HS) during the detection tries.
	 * This provides additional debounce that will help in the case
	 * of slow insertion.
	 * This also avoids the pause in audio due to accessory change
	 * from HP to HS
	 */
	if (ctx->hs_det_retry <= 0) {
		/* end of retries; report the status */
		snd_soc_jack_report(jack, jack_type, gpio->report);
	} else {
		/* Schedule another detection try if headphone or no jack
		 * is detected.During slow insertion of headset, first a
		 * headphone may be detected.
		 * Hence retry until headset is detected
		 */
		if (jack_type == SND_JACK_HEADSET) {
			ctx->hs_det_retry = 0;
			/* HS detected, no more retries needed */
			snd_soc_jack_report(jack, jack_type, gpio->report);
		} else {
			ctx->hs_det_retry--;
			schedule_delayed_work(&ctx->hs_insert_work,
				msecs_to_jiffies(ctx->hs_det_poll_intrvl));
			pr_info("%s:re-try hs detection after %d msec\n",
					__func__, ctx->hs_det_poll_intrvl);
		}
	}

	pr_info("Exit:%s\n", __func__);
	mutex_unlock(&ctx->jack_mlock);
}
/* Checks jack removal. */
static void cht_check_hs_remove_status(struct work_struct *work)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	struct cht_mc_private *ctx = container_of(work,
			struct cht_mc_private, hs_remove_work.work);
	int status = 0, jack_type = 0;

	/* Cancel any pending insertion detection.
	 * There could be pending insertion detection in the
	 * case of very slow insertion or insertion and immediate removal.
	 */
	cancel_delayed_work_sync(&ctx->hs_insert_work);

	mutex_lock(&ctx->jack_mlock);
	pr_info("Enter:%s\n", __func__);
	/* Initialize jack_type with previous status.
	 * If the event was an invalid one, we return the previous state
	 */
	jack_type = jack->status;

	if (jack->status) {
		/* jack is in connected state; look for removal event */
		status = cht_mc_check_jd_status(codec);
		if (status) {
			/* jd status high implies accessory disconnected */
			pr_info("Jack remove event\n");
			ctx->process_button_events = false;
			cancel_delayed_work_sync(&ctx->hs_button_en_work);
			status = rt5645_headset_detect(codec, false);
			jack_type = 0;
			cht_set_codec_power(codec, 0);

		} else if (((jack->status & SND_JACK_HEADSET) ==
				SND_JACK_HEADSET)
				&& !ctx->process_button_events) {
			/* Jack is still connected. We may come here if there
			 * was a spurious jack removal event. No state change
			 * is done until removal is confirmed by the
			 * check_jd_status above.i.e. jack status remains
			 * Headset or headphone.But as soon as the interrupt
			 * thread(cht_hs_detection) detected a jack removal,
			 * button processing gets disabled. Hence re-enable
			 * button processing in the case of headset.
			 */
			pr_info("spurious Jack remove event for headset\n");
			pr_info("re-enable button events\n");
			ctx->process_button_events = true;
		}
	}
	snd_soc_jack_report(jack, jack_type, gpio->report);
	pr_info("Exit:%s\n", __func__);
	mutex_unlock(&ctx->jack_mlock);
}
/* Check for button press/release */
static void cht_check_hs_button_status(struct work_struct *work)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	struct cht_mc_private *ctx = container_of(work,
			struct cht_mc_private, hs_button_work.work);
	int status = 0, jack_type = 0;
	int ret;

	mutex_lock(&ctx->jack_mlock);
	pr_info("Enter:%s", __func__);
	/* Initialize jack_type with previous status.
	 * If the event was an invalid one, we return the preious state
	 */
	jack_type = jack->status;

	if (((jack->status & SND_JACK_HEADSET) == SND_JACK_HEADSET)
			&& ctx->process_button_events) {

		status = cht_mc_check_jd_status(codec);
		if (!status) {
			/* confirm jack is connected */
			status = rt5645_button_detect(codec);
			if (jack->status & SND_JACK_BTN_0) {
				/* if button was previosly in pressed state*/
				if (!status) {
					pr_info("BR event received\n");
					jack_type = SND_JACK_HEADSET;
				}
			} else {
				/* If button was previously in released state */
				if (status) {
					pr_info("BP event received\n");
					jack_type = SND_JACK_HEADSET |
								SND_JACK_BTN_0;
				}
			}
		}
		/* There could be button interrupts during jack removal.
		 * There can be situations where a button interrupt is generated
		 * first but no jack removal interrupt is generated.
		 * This can happen on platforrms where jack detection is
		 * aligned to Headset Left pin instead of the ground  pin and
		 * codec multiplexes (ORs) the jack and button interrupts.
		 * So schedule a jack removal detection work
		 */
		ret = schedule_delayed_work(&ctx->hs_remove_work,
				msecs_to_jiffies(ctx->hs_remove_det_delay));
		if (!ret)
			pr_info("cht_check_hs_remove_status already queued\n");
		else
			pr_info("%s:Check hs removal after %d msec\n",
					__func__, ctx->hs_remove_det_delay);

	}
	snd_soc_jack_report(jack, jack_type, gpio->report);
	pr_info("Exit:%s\n", __func__);
	mutex_unlock(&ctx->jack_mlock);
}

/* Delayed work for enabling the overcurrent detection circuit
 * and interrupt for generating button events */
static void cht_enable_hs_button_events(struct work_struct *work)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio;
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	
	rt5645_en_button_detect(codec);
	/* TODO move the enable button event here */
}

static inline struct snd_soc_codec *cht_get_codec(struct snd_soc_card *card)
{
	bool found = false;
	struct snd_soc_codec *codec;

	list_for_each_entry(codec, &card->codec_dev_list, card_list) {
		if (!strstr(codec->name, "i2c-10EC5645:00")) {
			pr_info("cht_get_codec miss for codec %s \n", codec->name);
			continue;
		} else {
			pr_info("cht_get_codec hit codec was %s \n", codec->name);
			found = true;
			break;
		}
	}
	if (found == false) {
		pr_err("%s: cant find codec\n", __func__);
		return NULL;
	}
	return codec;
}

#define VLV2_PLAT_CLK_AUDIO	3
#define PLAT_CLK_FORCE_ON	1
#define PLAT_CLK_FORCE_OFF	2
static int platform_clock_control(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int  event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct snd_soc_codec *codec;

	codec = cht_get_codec(card);
	if (!codec) {
		pr_err("Codec not found; Unable to set platform clock\n");
		return -EIO;
	}
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO,
				PLAT_CLK_FORCE_ON);
		pr_info("Platform clk turned ON\n");
		snd_soc_codec_set_sysclk(codec, RT5645_SCLK_S_PLL1,
				0, CHT_PLAT_CLK_3_HZ, SND_SOC_CLOCK_IN);
	} else {
		/* Set codec clock source to internal clock before
		 * turning off the platform clock. Codec needs clock
		 * for Jack detection and button press
		 */
		snd_soc_codec_set_sysclk(codec, RT5645_SCLK_S_RCCLK,
				0, 0, SND_SOC_CLOCK_IN);
		vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO,
				PLAT_CLK_FORCE_OFF);
		pr_info("Platform clk turned OFF\n");
	}

	return 0;
}

static const struct snd_soc_dapm_widget cht_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
			platform_clock_control, SND_SOC_DAPM_PRE_PMU|
			SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route cht_audio_map[] = {
	{"IN1P", NULL, "Headset Mic"},
	{"IN1N", NULL, "Headset Mic"},
	//modify for amic xmys@20160108
	{"micbias2", NULL, "Int Mic"},
	{"IN2P", NULL, "micbias2"},
	{"IN2N", NULL, "micbias2"},
	//modify for amic xmys@20160108 end
//	{"DMIC L1", NULL, "Int Mic"},
//	{"DMIC R1", NULL, "Int Mic"},
	{"Headphone", NULL, "HPOL"},
	{"Headphone", NULL, "HPOR"},
	{"Ext Spk", NULL, "SPOL"},
	{"Ext Spk", NULL, "SPOR"},

//	{ "PLL1", NULL, "AIF1 Playback"},
	{ "AIF1 Playback", NULL, "ssp2 Tx"},
	{ "ssp2 Tx", NULL, "codec_out0"},
	{ "ssp2 Tx", NULL, "codec_out1"},
	{ "codec_in0", NULL, "ssp2 Rx" },
	{ "codec_in1", NULL, "ssp2 Rx" },
	{ "ssp2 Rx", NULL, "AIF1 Capture"},

	{ "ssp0 Tx", NULL, "modem_out"},
	{ "modem_in", NULL, "ssp0 Rx" },

	{ "ssp1 Tx", NULL, "bt_fm_out"},
	{ "bt_fm_in", NULL, "ssp1 Rx" },

	{"AIF1 Playback", NULL, "Platform Clock"},
	{"AIF1 Capture", NULL, "Platform Clock"},
	{"PLL1", NULL, "Platform Clock"},
	{"Headphone", NULL, "PLL1"},
	{"Ext Spk", NULL, "PLL1"},
};

static const struct snd_kcontrol_new cht_mc_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
};

#if 0
static int cht_cr_aif1_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int fmt;
	int ret;

	pr_info("Enter:%s, codec_dai_name:%s\n", __func__, codec_dai->name);

	/* proceed only if dai is valid */
	if (strncmp(codec_dai->name, "rt5645-aif1", 11))
		return 0;

	pr_info("%s, codec_name checked\n", __func__);

	/* TDM 4 slot 24 bit set the Rx and Tx bitmask to
	 * 4 active slots as 0xF
	 */
/*
	ret = snd_soc_dai_set_tdm_slot(codec_dai, 0, 0, 2,
			SNDRV_PCM_FORMAT_GSM);
	if (ret < 0) {
		pr_err("can't set codec TDM slot %d\n", ret);
		return ret;
	}
*/
	/* TDM slave Mode */
	fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBS_CFS;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_pll(codec_dai, 0, RT5645_PLL1_S_MCLK,
				  CHT_PLAT_CLK_3_HZ, params_rate(params) * 512);
	if (ret < 0) {
		pr_err("can't set codec pll: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, RT5645_SCLK_S_PLL1,
				     params_rate(params) * 512,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("can't set codec sysclk: %d\n", ret);
		return ret;
	}
	return 0;
}

#endif

static int cht_aif1_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int fmt;
	int ret;

	pr_info("Enter:%s, codec_dai_name:%s\n", __func__, codec_dai->name);

	/* proceed only if dai is valid */
	if (strncmp(codec_dai->name, "rt5645-aif1", 11))
		return 0;

	/* TDM 4 slot 24 bit set the Rx and Tx bitmask to
	 * 2 active slots as 0xF
	 */
/*
        pr_info("going to set tdm 0,0,2,24\n");
	ret = snd_soc_dai_set_tdm_slot(codec_dai, 0x0, 0x0, 2,
			SNDRV_PCM_FORMAT_GSM);
	if (ret < 0) {
		pr_err("can't set codec TDM slot %d\n", ret);
		return ret;
	}
*/

	/* TDM slave Mode */
//	fmt =   SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_IB_NF
//		| SND_SOC_DAIFMT_CBS_CFS;

	fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBS_CFS;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_pll(codec_dai, 0, RT5645_PLL1_S_MCLK,
				  CHT_PLAT_CLK_3_HZ, params_rate(params) * 512);
	if (ret < 0) {
		pr_err("can't set codec pll: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, RT5645_SCLK_S_PLL1,
				     params_rate(params) * 512,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("can't set codec sysclk: %d\n", ret);
		return ret;
	}
	return 0;
}

static int cht_compr_set_params(struct snd_compr_stream *cstream)
{
	return 0;
}

static const struct snd_soc_pcm_stream cht_dai_params = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static int cht_codec_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("Invoked %s for dailink %s\n", __func__, rtd->dai_link->name);

	/* The DSP will covert the FE rate to 48k, stereo, 24bits */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	/* set SSP2 to 24-bit */
	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
				    SNDRV_PCM_HW_PARAM_FIRST_MASK],
				    SNDRV_PCM_FORMAT_S24_LE);
	return 0;
}

static int cht_set_bias_level(struct snd_soc_card *card,
				struct snd_soc_dapm_context *dapm,
				enum snd_soc_bias_level level)
{
	int ret = 0;
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_OFF:
		card->dapm.bias_level = level;
		pr_info("card(%s)->bias_level %u\n", card->name,
				card->dapm.bias_level);
		break;
	default:
		pr_err("%s: Invalid bias level=%d\n", __func__, level);
		ret =  -EINVAL;
	}

	return ret;
}

static void cht_detect_hs_status(struct work_struct *work){
	cht_hs_detection(NULL);
}
#ifdef CHT_RT5645_JD_SOC
static irqreturn_t jd_soc_irq_handler(int irq, void *dev_id)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio;
	struct snd_soc_jack *jack = gpio->jack;
//	struct snd_soc_codec *codec = jack->codec;
	struct cht_mc_private *ctx = container_of(jack,
						struct cht_mc_private, jack);

	int ret = 0;

	ret = schedule_delayed_work(&ctx->hs_detect_work,
				msecs_to_jiffies(ctx->hs_det_delay));
		if (!ret)
			pr_info("cht_detect_hs_status already queued\n");
		else
			pr_info("%s: detect hs status after %d msec\n",
					__func__, ctx->hs_det_delay);

	return IRQ_HANDLED;
}
#endif
#ifdef TEST_JD_CODEC
static irqreturn_t jd_codec_irq_handler(int irq, void *dev_id)
{
	pr_info("%s, enter\n", __func__);
	return IRQ_HANDLED;
}
#endif

static int cht_audio_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	struct snd_soc_codec *codec;
	struct snd_soc_card *card = runtime->card;
	struct cht_mc_private *ctx = snd_soc_card_get_drvdata(runtime->card);
	int codec_gpio;
	int pol, val;
	struct gpio_desc *desc;

	pr_info("Enter:%s\n", __func__);
	codec = cht_get_codec(card);
	if (!codec) {
		pr_err("Codec not found; %s: failed\n", __func__);
		return -EIO;
	}
	/* Set codec bias level */
	cht_set_bias_level(card, &card->dapm, SND_SOC_BIAS_OFF);
	card->dapm.idle_bias_off = true;

//	snd_soc_update_bits(codec, RT5645_JD_CTRL,
//			RT5645_JD_MASK, RT5645_JD_JD1_IN4P);

	desc = devm_gpiod_get_index(codec->dev, NULL, 0);
	if (!IS_ERR(desc)) {
		codec_gpio = desc_to_gpio(desc);
		devm_gpiod_put(codec->dev, desc);

		ret = gpiod_export(desc, true);
		if (ret)
			pr_info("%s: Unable to export GPIO%d (JD/BP)! Returned %d.\n",
					__func__, codec_gpio, ret);
		pol = gpiod_is_active_low(desc);
		val = gpiod_get_value(desc);
		pr_info("%s: GPIOs - JD/BP-int: %d (pol = %d, val = %d)\n",
				__func__, codec_gpio, pol, val);

	} else {
		codec_gpio = -1;
		pr_err("%s: GPIOs - JD/BP-int: Not present!\n", __func__);
	}


	hs_gpio.gpio = codec_gpio;

	ctx->intr_debounce = CHT_INTR_DEBOUNCE;
	ctx->hs_det_delay = CHT_HS_DET_DELAY;
	ctx->hs_insert_det_delay = CHT_HS_INSERT_DET_DELAY;
	ctx->hs_remove_det_delay = CHT_HS_REMOVE_DET_DELAY;
	ctx->button_det_delay = CHT_BUTTON_DET_DELAY;
	ctx->hs_det_poll_intrvl = CHT_HS_DET_POLL_INTRVL;
	ctx->hs_det_retry = CHT_HS_DET_RETRY_COUNT;
	ctx->button_en_delay = CHT_BUTTON_EN_DELAY;
	ctx->process_button_events = false;

	INIT_DELAYED_WORK(&ctx->hs_detect_work, cht_detect_hs_status);
	INIT_DELAYED_WORK(&ctx->hs_insert_work, cht_check_hs_insert_status);
	INIT_DELAYED_WORK(&ctx->hs_remove_work, cht_check_hs_remove_status);
	INIT_DELAYED_WORK(&ctx->hs_button_work, cht_check_hs_button_status);
	INIT_DELAYED_WORK(&ctx->hs_button_en_work, cht_enable_hs_button_events);

	mutex_init(&ctx->jack_mlock);

	ret = snd_soc_jack_new(codec, "Intel MID Audio Jack",
			       SND_JACK_HEADSET | SND_JACK_HEADPHONE |
			       SND_JACK_BTN_0, &ctx->jack);
	if (ret) {
		pr_err("jack creation failed\n");
		return ret;
	}

	snd_jack_set_key(ctx->jack.jack, SND_JACK_BTN_0, KEY_MEDIA);

#ifdef TEST_JD_CODEC
	gpio_request(codec_gpio, "JD_CODEC_N");
	gpio_direction_input(codec_gpio);
	ret = request_any_context_irq(gpio_to_irq(codec_gpio), jd_codec_irq_handler,
            IRQF_DISABLED| IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "jd_codec_irq", NULL);
	if (ret != 1) {
		pr_info("unable to get jd_codec IRQ\n");
	}else{
		pr_info("gpio_to_irq(307) = %d", gpio_to_irq(codec_gpio));
	}
#endif


#ifndef TEST_JD_CODEC
	ret = snd_soc_jack_add_gpios(&ctx->jack, 1, &hs_gpio);
	if (ret) {
		pr_err("adding jack GPIO failed\n");
		return ret;
	}
#endif
#ifdef CHT_RT5645_JD_SOC
	gpio_request(CHT_RT5645_JD_SOC, "JD_SOC_N");
	gpio_direction_input(CHT_RT5645_JD_SOC);

	ret = request_any_context_irq(gpio_to_irq(CHT_RT5645_JD_SOC), jd_soc_irq_handler, IRQF_DISABLED| IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "jd_soc_irq", NULL);
	if (ret != 1) {
		pr_info("unable to get jd_soc IRQ\n");
	}else{
		pr_info("gpio_to_irq(406) = %d", gpio_to_irq(CHT_RT5645_JD_SOC));
	}
#endif
	/* Keep the voice call paths active during
	 * suspend. Mark the end points ignore_suspend
	 */
	/*TODO: CHECK this */
	snd_soc_dapm_ignore_suspend(&codec->dapm, "HPOL");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "HPOR");

	snd_soc_dapm_ignore_suspend(&codec->dapm, "SPOL");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "SPOR");

	snd_soc_dapm_enable_pin(&card->dapm, "Headset Mic");
	snd_soc_dapm_enable_pin(&card->dapm, "Headphone");
	snd_soc_dapm_enable_pin(&card->dapm, "Ext Spk");
	snd_soc_dapm_enable_pin(&card->dapm, "Int Mic");

	snd_soc_dapm_enable_pin(&card->dapm, "JD Power");

	snd_soc_dapm_sync(&card->dapm);
	vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO,
                                PLAT_CLK_FORCE_ON);
        pr_info("Platform clk turned ON\n");
        snd_soc_codec_set_sysclk(codec, RT5645_SCLK_S_PLL1,
                                0, CHT_PLAT_CLK_3_HZ, SND_SOC_CLOCK_IN);
	return ret;
}

static unsigned int rates_8000_16000[] = {
	8000,
	16000,
};

static struct snd_pcm_hw_constraint_list constraints_8000_16000 = {
	.count = ARRAY_SIZE(rates_8000_16000),
	.list  = rates_8000_16000,
};

static unsigned int rates_48000[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count = ARRAY_SIZE(rates_48000),
	.list  = rates_48000,
};

static int cht_aif1_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_48000);
}

/*
static struct snd_soc_ops cht_cr_aif1_ops = {
	.startup = cht_aif1_startup,
	.hw_params = cht_cr_aif1_hw_params,
};
*/

static struct snd_soc_ops cht_aif1_ops = {
	.startup = cht_aif1_startup,
};

static int cht_8k_16k_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_8000_16000);
}

static struct snd_soc_ops cht_8k_16k_ops = {
	.startup = cht_8k_16k_startup,
	.hw_params = cht_aif1_hw_params,
};
static struct snd_soc_ops cht_be_ssp2_ops = {
	.hw_params = cht_aif1_hw_params,
};

static struct snd_soc_compr_ops cht_compr_ops = {
	.set_params = cht_compr_set_params,
};

static struct snd_soc_dai_link cht_dailink[] = {
// link cpu_dai and codec dai here
// for bring up perpose
	[CHT_DPCM_AUDIO] = {
		.name = "Cherrytrail Audio Port",
		.stream_name = "Cherrytrail Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-platform",
		.init = cht_audio_init,
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &cht_aif1_ops,
//		.ops = &cht_cr_aif1_ops,
	},
	[CHT_DPCM_DB] = {
		.name = "Cherrytrail DB Audio Port",
		.stream_name = "Deep Buffer Audio",
		.cpu_dai_name = "Deepbuffer-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-platform",
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &cht_aif1_ops,
		.dpcm_playback = 1,
	},
	[CHT_DPCM_COMPR] = {
		.name = "Cherrytrail Compressed Port",
		.stream_name = "Cherrytrail Compress",
		.cpu_dai_name = "Compress-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-platform",
		.ignore_suspend = 1,
		.dynamic = 1,
		.compr_ops = &cht_compr_ops,
		.dpcm_playback = 1,
	},
	[CHT_DPCM_VOIP] = {
		.name = "Cherrytrail VOIP Port",
		.stream_name = "Cherrytrail Voip",
		.cpu_dai_name = "Voip-cpu-dai",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &cht_8k_16k_ops,
		.dynamic = 1,
	},
	[CHT_DPCM_LL] = {
		.name = "Cherrytrail LL Audio Port",
		.stream_name = "Low Latency Audio",
		.cpu_dai_name = "Lowlatency-cpu-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "sst-platform",
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &cht_aif1_ops,
	},
	[CHT_DPCM_PROBE] = {
		.name = "Cherrytrail Probe Port",
		.stream_name = "Cherrytrail Probe",
		.cpu_dai_name = "Probe-cpu-dai",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.playback_count = 8,
		.capture_count = 8,
	},
	/* CODEC<->CODEC link */
	{
		.name = "Cherrytrail Codec-Loop Port",
		.stream_name = "Cherrytrail Codec-Loop",
		.cpu_dai_name = "ssp2-port",
		.platform_name = "sst-platform",
		.codec_dai_name = "rt5645-aif1",
		.codec_name = "i2c-10EC5645:00",
		.dai_fmt = SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_IB_NF
			| SND_SOC_DAIFMT_CBS_CFS,
		.params = &cht_dai_params,
		.dsp_loopback = true,
	},
	{
		.name = "Cherrytrail Modem-Loop Port",
		.stream_name = "Cherrytrail Modem-Loop",
		.cpu_dai_name = "ssp0-port",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.params = &cht_dai_params,
		.dsp_loopback = true,
	},
	{
		.name = "Cherrytrail BTFM-Loop Port",
		.stream_name = "Cherrytrail BTFM-Loop",
		.cpu_dai_name = "ssp1-port",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.params = &cht_dai_params,
		.dsp_loopback = true,
	},
	/* Back ends */
	{
		.name = "SSP2-Codec",
		.be_id = 1,
		.cpu_dai_name = "ssp2-port",
		.platform_name = "sst-platform",
		.no_pcm = 1,
		.codec_dai_name = "rt5645-aif1",
		.codec_name = "i2c-10EC5645:00",
		.be_hw_params_fixup = cht_codec_fixup,
		.ignore_suspend = 1,
		.ops = &cht_be_ssp2_ops,
	},
	{
		.name = "SSP1-BTFM",
		.be_id = 2,
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd-soc-dummy",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
	},
	{
		.name = "SSP0-Modem",
		.be_id = 3,
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd-soc-dummy",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
	},
};

#ifdef CONFIG_PM_SLEEP
static int snd_cht_prepare(struct device *dev)
{
	pr_debug("In %s device name\n", __func__);
	return snd_soc_suspend(dev);
}

static void snd_cht_complete(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO, PLAT_CLK_FORCE_OFF);
	snd_soc_resume(dev);
}

static int snd_cht_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	return snd_soc_poweroff(dev);
}
#else
#define snd_cht_prepare NULL
#define snd_cht_complete NULL
#define snd_cht_poweroff NULL
#endif

/* SoC card */
static struct snd_soc_card snd_soc_card_cht = {
	.name = "cherrytrailaud",
	.dai_link = cht_dailink,
	.num_links = ARRAY_SIZE(cht_dailink),
	.set_bias_level = cht_set_bias_level,
	.dapm_widgets = cht_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cht_dapm_widgets),
	.dapm_routes = cht_audio_map,
	.num_dapm_routes = ARRAY_SIZE(cht_audio_map),
	.controls = cht_mc_controls,
	.num_controls = ARRAY_SIZE(cht_mc_controls),
};

static int snd_cht_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct cht_mc_private *drv;

	pr_info(_ML "snd_cht_mc_probe\n");

	pr_info(_ML "Entry %s\n", __func__);

	/* Audio Platform clock is on by default. The machine driver requests
	 * this clock to be turned ON and OFF on playing any stream. But
	 * until any stream is played the clock remains ON. Hence request the
	 * clock to be turned OFF initially.
	 */
	vlv2_plat_configure_clock(VLV2_PLAT_CLK_AUDIO, PLAT_CLK_FORCE_OFF);

	drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_ATOMIC);
	if (!drv) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}

	/* register the soc card */
	snd_soc_card_cht.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&snd_soc_card_cht, drv);
	ret_val = snd_soc_register_card(&snd_soc_card_cht);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		return ret_val;
	}
	platform_set_drvdata(pdev, &snd_soc_card_cht);
	pr_info("%s successful\n", __func__);
	return ret_val;
}

static void snd_cht_unregister_jack(struct cht_mc_private *ctx)
{
	/* Set process button events to false so that the button
	   delayed work will not be scheduled.*/
	ctx->process_button_events = false;
	cancel_delayed_work_sync(&ctx->hs_detect_work);
	cancel_delayed_work_sync(&ctx->hs_insert_work);
	cancel_delayed_work_sync(&ctx->hs_button_en_work);
	cancel_delayed_work_sync(&ctx->hs_button_work);
	cancel_delayed_work_sync(&ctx->hs_remove_work);
	snd_soc_jack_free_gpios(&ctx->jack, 1, &hs_gpio);
#ifdef CHT_RT5645_JD_SOC
	gpio_free(CHT_RT5645_JD_SOC);
#endif
}

static int snd_cht_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct cht_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	snd_cht_unregister_jack(drv);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void snd_cht_mc_shutdown(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct cht_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	snd_cht_unregister_jack(drv);
}

const struct dev_pm_ops snd_cht_mc_pm_ops = {
	.prepare = snd_cht_prepare,
	.complete = snd_cht_complete,
	.poweroff = snd_cht_poweroff,
};


static const struct acpi_device_id cht_mc_acpi_ids[] = {
	{ "AMCR22A8", 0 },
	{ "TIMC22A8", 0 },
	{},
};
MODULE_DEVICE_TABLE(acpi, cht_mc_acpi_ids);

static struct platform_driver snd_cht_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "cht_rt5645",
		.pm = &snd_cht_mc_pm_ops,
		.acpi_match_table = ACPI_PTR(cht_mc_acpi_ids),
	},
	.probe = snd_cht_mc_probe,
	.remove = snd_cht_mc_remove,
	.shutdown = snd_cht_mc_shutdown,
};

static int __init snd_cht_driver_init(void)
{
	int ret = 0;
	pr_info("Cherrytrail Machine Driver cht_rt5645 register begin\n");
	ret = platform_driver_register(&snd_cht_mc_driver);
	pr_info("Cherrytrail Machine Driver cht_rt5645 register end\n");

	if (ret)
		pr_err("%s: Failed to register cht_rt5645 Machine driver!\n",
			__func__);
	else
		pr_info("%s: cht_rt5645 Machine driver registered.\n",
			__func__);

	return ret;
}
late_initcall(snd_cht_driver_init);

static void __exit snd_cht_driver_exit(void)
{
	pr_info("In %s\n", __func__);
	platform_driver_unregister(&snd_cht_mc_driver);
}
module_exit(snd_cht_driver_exit);

MODULE_DESCRIPTION("ASoC Intel(R) Cherrytrail Machine driver");
MODULE_AUTHOR("Mythri P K <mythri.p.k@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:cht_rt5645");
