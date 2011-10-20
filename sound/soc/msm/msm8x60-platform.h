
int mvs_pcm_trigger(struct snd_pcm_substream *substream, int cmd);
int mvs_pcm_open(struct snd_pcm_substream *substream);
int mvs_pcm_copy(struct snd_pcm_substream *substream, int a,
	 snd_pcm_uframes_t hwoff, void __user *buf, snd_pcm_uframes_t frames);
int mvs_pcm_close(struct snd_pcm_substream *substream);

int mvs_pcm_prepare(struct snd_pcm_substream *substream);
snd_pcm_uframes_t mvs_pcm_pointer(struct snd_pcm_substream *substream);
int mvs_pcm_mmap(struct snd_pcm_substream *substream, struct vm_area_struct *vma);
int mvs_pcm_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params);

int mvs_pcm_remove(struct platform_device *devptr);
int mvs_pcm_new(struct snd_card *card, struct snd_soc_dai *codec_dai, struct snd_pcm *pcm);

int msm_dsp_trigger(struct snd_pcm_substream *substream, int cmd);
int msm_dsp_open(struct snd_pcm_substream *substream);
int msm_dsp_copy(struct snd_pcm_substream *substream, int a,
	 snd_pcm_uframes_t hwoff, void __user *buf, snd_pcm_uframes_t frames);
int msm_dsp_close(struct snd_pcm_substream *substream);

int msm_dsp_prepare(struct snd_pcm_substream *substream);
snd_pcm_uframes_t msm_dsp_pointer(struct snd_pcm_substream *substream);
int msm_dsp_mmap(struct snd_pcm_substream *substream, struct vm_area_struct *vma);
int msm_dsp_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params);

int msm_dsp_remove(struct platform_device *devptr);
int msm_dsp_new(struct snd_card *card, struct snd_soc_dai *codec_dai, struct snd_pcm *pcm);

