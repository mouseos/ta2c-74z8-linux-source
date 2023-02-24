/*
 * HWDEP Interface for HD-audio codec
 *
 * Copyright (c) 2007 Takashi Iwai <tiwai@suse.de>
 *
 *  This driver is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This driver is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/init.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <sound/core.h>
#include <sound/hda_codec.h>
#include <sound/hda_hwdep.h>
#include <sound/minors.h>

/*
 * write/read an out-of-bound verb
 */
static int verb_write_ioctl(struct hda_codec *codec,
			    struct hda_verb_ioctl __user *arg)
{
	u32 verb, res;

	if (get_user(verb, &arg->verb))
		return -EFAULT;
	res = snd_hda_codec_read(codec, verb >> 24, 0,
				 (verb >> 8) & 0xffff, verb & 0xff);
	if (put_user(res, &arg->res))
		return -EFAULT;
	return 0;
}

static int get_wcap_ioctl(struct hda_codec *codec,
			  struct hda_verb_ioctl __user *arg)
{
	u32 verb, res;
	
	if (get_user(verb, &arg->verb))
		return -EFAULT;
	res = get_wcaps(codec, verb >> 24);
	if (put_user(res, &arg->res))
		return -EFAULT;
	return 0;
}


/*
 */
static int hda_hwdep_ioctl(struct snd_hwdep *hw, struct file *file,
			   unsigned int cmd, unsigned long arg)
{
	struct hda_codec *codec = hw->private_data;
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case HDA_IOCTL_PVERSION:
		return put_user(HDA_HWDEP_VERSION, (int __user *)argp);
	case HDA_IOCTL_VERB_WRITE:
		return verb_write_ioctl(codec, argp);
	case HDA_IOCTL_GET_WCAP:
		return get_wcap_ioctl(codec, argp);
	}
	return -ENOIOCTLCMD;
}

#ifdef CONFIG_COMPAT
static int hda_hwdep_ioctl_compat(struct snd_hwdep *hw, struct file *file,
				  unsigned int cmd, unsigned long arg)
{
	return hda_hwdep_ioctl(hw, file, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static int hda_hwdep_open(struct snd_hwdep *hw, struct file *file)
{
#ifndef CONFIG_SND_DEBUG_VERBOSE
	if (!capable(CAP_SYS_RAWIO))
		return -EACCES;
#endif
	return 0;
}

int snd_hda_create_hwdep(struct hda_codec *codec)
{
	char hwname[16];
	struct snd_hwdep *hwdep;
	int err;

	sprintf(hwname, "HDA Codec %d", codec->addr);
	err = snd_hwdep_new(codec->card, hwname, codec->addr, &hwdep);
	if (err < 0)
		return err;
	codec->hwdep = hwdep;
	sprintf(hwdep->name, "HDA Codec %d", codec->addr);
	hwdep->iface = SNDRV_HWDEP_IFACE_HDA;
	hwdep->private_data = codec;
	hwdep->exclusive = 1;
/* FIXME: Enable after moving to 3.15 */
#if 0
	hwdep->groups = snd_hda_dev_attr_groups;
#endif
	hwdep->ops.open = hda_hwdep_open;
	hwdep->ops.ioctl = hda_hwdep_ioctl;
#ifdef CONFIG_COMPAT
	hwdep->ops.ioctl_compat = hda_hwdep_ioctl_compat;
#endif

	/* link to codec */
/* FIXME: Enable after moving to 3.15 */
#if 0
	hwdep->dev = codec->dev;
#endif
	return 0;
}
