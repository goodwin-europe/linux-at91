/*
 * sama5d3_aic32x4 - SoC audio for SAMA5D based boards
 *                    which use TLV320AIC3204 as codec.
 *
 * Copyright (C) 2014 Goodwin JSC
 *
 * Author: Alexander Morozov <etesial@gmail.com>
 * Based on sama5_aic32x4.c by Bo Shen
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

#include "atmel_ssc_dai.h"

#define MCLK_RATE (12000000)

static struct clk *mclk;

static const struct snd_soc_dapm_widget sama5_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("External Mic", NULL),
};

static int sama5d3_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;
	u32 dai_format;

	// dai_format = SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_NB_NF |
	// dai_format = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF |
		//SND_SOC_DAIFMT_CBS_CFS;

	dai_format = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM;

	printk("sama5d3_hw_params()\n");
	/* set codec DAI configuration */
	snd_soc_dai_set_fmt(codec_dai, dai_format);

	/* set cpu DAI configuration */
	snd_soc_dai_set_fmt(cpu_dai, dai_format);

	ret = snd_soc_dai_set_sysclk(codec_dai, 0,
				     MCLK_RATE, SND_SOC_CLOCK_OUT);
	if (ret) {
		pr_err("%s: failed setting codec sysclk\n", __func__);
		return ret;
	}

	return 0;
}

static struct snd_soc_ops sama5d3_soc_ops = {
	.hw_params = sama5d3_hw_params,
};

int sama5d3_snd_suspend_pre(struct snd_soc_card *card)
{
	printk("samad5 clk suspend\n");
	clk_disable(mclk);
	return 0;
}

int sama5d3_snd_resume_pre(struct snd_soc_card *card)
{
	clk_enable(mclk);
	return 0;
}

static struct snd_soc_dai_link sama5d3_dai = {
	.name = "AIC32x4",
	.stream_name = "AIC32x4",
	.codec_dai_name = "tlv320aic32x4-hifi",
	.dai_fmt = SND_SOC_DAIFMT_DSP_A
		| SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBM_CFM,
	.ops = &sama5d3_soc_ops,
};

static struct snd_soc_card snd_soc_sama5d3 = {
	.name = "AIC32x4 @ SAMA5D3",
	.dai_link = &sama5d3_dai,
	.num_links = 1,
	.suspend_pre = sama5d3_snd_suspend_pre,
	.resume_pre = sama5d3_snd_resume_pre,
	.dapm_widgets = sama5_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(sama5_dapm_widgets),
	.fully_routed = true,
};

static int sama5d3_aic32x4_dt_init(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *codec_np, *cpu_np;
	struct snd_soc_card *card = &snd_soc_sama5d3;
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
	sama5d3_dai.cpu_of_node = cpu_np;
	sama5d3_dai.platform_of_node = cpu_np;
	of_node_put(cpu_np);

	codec_np = of_parse_phandle(np, "atmel,audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "failed to get codec info\n");
		ret = -EINVAL;
		return ret;
	}
	sama5d3_dai.codec_of_node = codec_np;
	of_node_put(codec_np);

	return 0;
}

static int sama5d3_aic32x4_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_sama5d3;
	struct clk *clk_src, *clk_prog;
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
	ret = sama5d3_aic32x4_dt_init(pdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to init dt info\n");
		goto err_set_audio;
	}

	dev_info(&pdev->dev, "get pck");
	mclk = clk_get(&pdev->dev, "pck");
	if (IS_ERR(mclk)) {
		dev_err(&pdev->dev, "failed to get pck0\n");
		ret = PTR_ERR(mclk);
		goto err_set_audio;
	}

	dev_info(&pdev->dev, "get prog");
	clk_prog = clk_get(&pdev->dev, "prog");
	if (IS_ERR(mclk)) {
		dev_err(&pdev->dev, "failed to get pck0\n");
		ret = PTR_ERR(mclk);
		goto err_set_audio;
	}

	dev_info(&pdev->dev, "get main");
	clk_src = clk_get(&pdev->dev, "main");
	if (IS_ERR(clk_src)) {
		dev_err(&pdev->dev, "failed to get main\n");
		ret = PTR_ERR(clk_src);
		goto err_set_audio;
	}

	dev_info(&pdev->dev, "set parent");
	ret = clk_set_parent(clk_prog, clk_src);
	clk_put(clk_src);
	if (ret != 0) {
		dev_err(&pdev->dev, "failed to set MCLK parent\n");
		goto err_set_audio;
	}

	dev_info(&pdev->dev, "setting pck0 to %dHz\n", MCLK_RATE);

	clk_set_rate(mclk, MCLK_RATE);

	dev_info(&pdev->dev, "enable clk_prog");
	clk_prepare(clk_prog);
	clk_enable(clk_prog);

	dev_info(&pdev->dev, "prepare mclk");
	clk_prepare(mclk);
	clk_prepare(mclk);
	clk_prepare(mclk);
	dev_info(&pdev->dev, "enable mclk");
	clk_enable(mclk);
	clk_enable(mclk);
	clk_enable(mclk);
	dev_info(&pdev->dev, "register card");

	snd_soc_register_card(card);

	dev_info(&pdev->dev, "enable mclk");
	return 0;

err_set_audio:
	atmel_ssc_put_audio(0);
	return ret;
}

static int sama5d3_aic32x4_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	clk_disable(mclk);
	snd_soc_unregister_card(card);
	atmel_ssc_put_audio(0);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sama5d3_aic32x4_dt_ids[] = {
	{ .compatible = "atmel,sama5d3-aic32x4", },
	{ }
};
#endif

static struct platform_driver sama5d3_aic32x4_driver = {
	.driver = {
		.name = "sama5d3-aic32x4-audio",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(sama5d3_aic32x4_dt_ids),
	},
	.probe = sama5d3_aic32x4_probe,
	.remove = sama5d3_aic32x4_remove,
};

module_platform_driver(sama5d3_aic32x4_driver);

/* Module information */
MODULE_AUTHOR("Alexander Morozov <linux@meltdown.ru>");
MODULE_DESCRIPTION("ALSA SoC machine driver for SAMA5D3 - TLV320AIC32x4");
MODULE_LICENSE("GPL");
