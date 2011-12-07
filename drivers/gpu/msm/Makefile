ccflags-y := -Iinclude/drm

msm_kgsl_core-$(CONFIG_MSM_KGSL) = \
	kgsl.o \
	kgsl_log.o \
	kgsl_mmu.o \
	kgsl_sharedmem.o \
	kgsl_pwrctrl.o \
	kgsl_postmortem.o

msm_kgsl_a200-$(CONFIG_MSM_KGSL) += \
	kgsl_ringbuffer.o \
	kgsl_drawctxt.o \
	kgsl_cmdstream.o \
	kgsl_yamato.o

msm_kgsl_z180-$(CONFIG_MSM_KGSL_2D) += \
	kgsl_g12_drawctxt.o \
	kgsl_g12_cmdstream.o \
	kgsl_g12.o

msm_kgsl_core-$(CONFIG_MSM_KGSL_CFF_DUMP) += kgsl_cffdump.o
msm_kgsl_core-$(CONFIG_MSM_KGSL_DRM) += kgsl_drm.o

msm_kgsl_core-objs = $(msm_kgsl_core-y)
msm_kgsl_a200-objs = $(msm_kgsl_a200-y)
msm_kgsl_z180-objs = $(msm_kgsl_z180-y)

obj-$(CONFIG_MSM_KGSL) += \
	msm_kgsl_core.o \
	msm_kgsl_a200.o

obj-$(CONFIG_MSM_KGSL_2D) += msm_kgsl_z180.o
