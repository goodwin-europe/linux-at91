/*
 * sama5d3ek_sgtl5000 - SoC audio for SAMA5D3EK based boards
 *                    with audio shield based on SGTL5000
 *
 * Author: Alexander Morozov <linux@meltdown.ru>
 * Based on sama5d3_wm8904.c by Bo Shen
 *
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>

#include "../codecs/sgtl5000.h"
#include "atmel_ssc_dai.h"

#define MCLK_RATE 12000000

static struct clk *mclk;

static const struct snd_soc_dapm_widget sama5ek_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Line Out Jack", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
};

static int sama5d3_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, SGTL5000_SYSCLK,
		MCLK_RATE, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("%s - Failed to set SGTL5000 SYSCLK\n", __func__);
		return ret;
	}

	return 0;
}

static struct snd_soc_ops sama5d3_soc_ops = {
	.hw_params = sama5d3_hw_params,
};

int sama5d3ek_snd_suspend_pre(struct snd_soc_card *card)
{
	clk_disable(mclk);
	return 0;
}

int sama5d3ek_snd_resume_pre(struct snd_soc_card *card)
{
	clk_enable(mclk);
	return 0;
}

static struct snd_soc_dai_link sama5d3ek_dai = {
	.name = "SGTL5000",
	.stream_name = "SGTL5000 PCM",
	.codec_dai_name = "sgtl5000",
	.dai_fmt = SND_SOC_DAIFMT_I2S
		| SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBM_CFM,
	.ops = &sama5d3_soc_ops,
};

static struct snd_soc_card snd_soc_sama5d3ek = {
	.name = "SGTL5000 @ SAMA5D3",
	.dai_link = &sama5d3ek_dai,
	.num_links = 1,
	.suspend_pre = sama5d3ek_snd_suspend_pre,
	.resume_pre = sama5d3ek_snd_resume_pre,
	.dapm_widgets = sama5ek_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(sama5ek_dapm_widgets),
	.fully_routed = true,
};

static int sama5d3ek_sgtl5000_dt_init(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *codec_np, *cpu_np;
	struct snd_soc_card *card = &snd_soc_sama5d3ek;
	int ret;

	if (!np)
		return -1;

	ret = snd_soc_of_parse_card_name(card, "atmel,model");
	if (ret) {
		dev_err(&pdev->dev, "failed to parse card name\n");
		return ret;
	}

	ret = snd_soc_of_parse_audio_routing(card, "atmel,audio-routing");
	if (ret) {
		dev_err(&pdev->dev, "failed to parse audio routing\n");
		return ret;
	}

	cpu_np = of_parse_phandle(np, "atmel,ssc-controller", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "failed to get dai and pcm info\n");
		ret = -EINVAL;
		return ret;
	}
	sama5d3ek_dai.cpu_of_node = cpu_np;
	sama5d3ek_dai.platform_of_node = cpu_np;
	of_node_put(cpu_np);

	codec_np = of_parse_phandle(np, "atmel,audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "failed to get codec info\n");
		ret = -EINVAL;
		return ret;
	}
	sama5d3ek_dai.codec_of_node = codec_np;
	of_node_put(codec_np);

	return 0;
}

static int sama5d3ek_sgtl5000_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_sama5d3ek;
	struct clk *clk_src;
	struct pinctrl *pinctrl;
	int ret;

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "failed to request pinctrl\n");
		return PTR_ERR(pinctrl);
	}

	ret = atmel_ssc_set_audio(0);
	if (ret != 0) {
		dev_err(&pdev->dev, "failed to set SSC 0 for audio\n");
		return ret;
	}

	card->dev = &pdev->dev;
	ret = sama5d3ek_sgtl5000_dt_init(pdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to init dt info\n");
		goto err_set_audio;
	}

	mclk = clk_get(NULL, "pck1");
	if (IS_ERR(mclk)) {
		dev_err(&pdev->dev, "failed to get pck1\n");
		ret = PTR_ERR(mclk);
		goto err_set_audio;
	}

	clk_src = clk_get(NULL, "main");
	if (IS_ERR(clk_src)) {
		dev_err(&pdev->dev, "failed to get 'main' clk\n");
		ret = PTR_ERR(clk_src);
		goto err_set_audio;
	}

	ret = clk_set_parent(mclk, clk_src);
	clk_put(clk_src);
	if (ret != 0) {
		dev_err(&pdev->dev, "failed to set MCLK parent\n");
		goto err_set_audio;
	}
	
	dev_info(&pdev->dev, "setting pck0 to %dHz\n", MCLK_RATE);

	clk_set_rate(mclk, MCLK_RATE);
	clk_enable(mclk);

	snd_soc_register_card(card);

	return 0;

err_set_audio:
	atmel_ssc_put_audio(0);
	return ret;
}

static int sama5d3ek_sgtl5000_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	clk_disable(mclk);
	snd_soc_unregister_card(card);
	atmel_ssc_put_audio(0);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sama5d3ek_sgtl5000_dt_ids[] = {
	{ .compatible = "atmel,sama5d3ek-sgtl5000", },
	{ }
};
#endif

static struct platform_driver sama5d3ek_sgtl5000_driver = {
	.driver = {
		.name = "sama5d3ek-audio",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sama5d3ek_sgtl5000_dt_ids),
	},
	.probe = sama5d3ek_sgtl5000_probe,
	.remove = sama5d3ek_sgtl5000_remove,
};

module_platform_driver(sama5d3ek_sgtl5000_driver);

/* Module information */
MODULE_AUTHOR("Alexander Morozov <linux@meltdown.ru>");
MODULE_DESCRIPTION("ALSA SoC machine driver for SAMA5D3 - SGTL5000");
MODULE_LICENSE("GPL");
