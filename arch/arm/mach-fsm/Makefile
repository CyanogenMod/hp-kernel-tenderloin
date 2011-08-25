obj-y += io.o dma.o memory.o
obj-y += clock.o
obj-y += modem_notifier.o
obj-$(CONFIG_CPU_FREQ_MSM) += cpufreq.o
obj-$(CONFIG_DEBUG_FS) += nohlt.o clock-debug.o

obj-$(CONFIG_MSM_PROC_COMM) += proc_comm.o
ifndef CONFIG_ARCH_MSM8X60
	obj-$(CONFIG_MSM_PROC_COMM) += clock-pcom.o internal_power_rail.o
	obj-$(CONFIG_MSM_PROC_COMM) += vreg.o mpp.o
	ifdef CONFIG_MSM_PROC_COMM
		obj-$(CONFIG_DEBUG_FS) += pmic_debugfs.o
	endif
endif

obj-$(CONFIG_ARCH_MSM_ARM11) += acpuclock.o timer.o
obj-$(CONFIG_ARCH_MSM_SCORPION) += timer.o
obj-$(CONFIG_ARCH_MSM_SCORPIONMP) += timer.o

obj-$(CONFIG_ARCH_MSM_SCORPION) += msm_fault_handlers.o

obj-$(CONFIG_MSM_VIC) += irq-vic.o

ifdef CONFIG_ARCH_QSD8X50
	obj-$(CONFIG_MSM_SOC_REV_NONE) += acpuclock-8x50.o
	obj-$(CONFIG_MSM_SOC_REV_A) += acpuclock-8x50a.o
endif

obj-$(CONFIG_SMP) += headsmp.o platsmp.o

obj-$(CONFIG_MSM_CPU_AVS) += avs.o avs_hw.o
obj-$(CONFIG_CPU_V6) += idle-v6.o
obj-$(CONFIG_CPU_V7) += idle-v7.o
obj-$(CONFIG_MSM_JTAG_V7) += jtag-v7.o


quiet_cmd_mkrpcsym = MKCAP   $@
      cmd_mkrpcsym = $(PERL) $(srctree)/$(src)/mkrpcsym.pl $< $@

target += smd_rpc_sym.c
$(obj)/smd_rpc_sym.c: $(src)/smd_rpc_sym $(src)/mkrpcsym.pl
	$(call if_changed,mkrpcsym)

obj-$(CONFIG_MSM_PIL) += peripheral-reset.o
obj-$(CONFIG_MSM_PIL) += peripheral-loader.o
obj-$(CONFIG_ARCH_QSD8X50) += sirc.o
obj-$(CONFIG_MSM_FIQ_SUPPORT) += fiq_glue.o
obj-$(CONFIG_MACH_TROUT) += board-trout-rfkill.o
obj-$(CONFIG_MSM_SDIO_AL) += sdio_al.o
obj-$(CONFIG_MSM_SDIO_AL_TEST) += sdio_al_test.o
obj-$(CONFIG_MSM_SMD) += smd.o smd_debug.o remote_spinlock.o socinfo.o
ifndef CONFIG_ARCH_MSM8X60
	obj-$(CONFIG_MSM_SMD) += nand_partitions.o pmic.o
	obj-$(CONFIG_MSM_ONCRPCROUTER) += rpc_hsusb.o rpc_pmapp.o rpc_fsusb.o
	obj-$(CONFIG_MSM_SMD_LOGGING) += smem_log.o
endif
obj-$(CONFIG_MSM_SMD_TTY) += smd_tty.o
obj-$(CONFIG_MSM_SMD_QMI) += smd_qmi.o
obj-$(CONFIG_MSM_SMD_PKT) += smd_pkt.o
obj-$(CONFIG_MSM_SDIO_CTL) += sdio_ctl.o
obj-$(CONFIG_MSM_SMD_NMEA) += smd_nmea.o
obj-$(CONFIG_MSM_RESET_MODEM) += reset_modem.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_device.o
obj-$(CONFIG_DEBUG_FS) += smd_rpc_sym.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_clients.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_xdr.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += rpcrouter_smd_xprt.o
obj-$(CONFIG_MSM_RPC_SDIO_XPRT) += rpcrouter_sdio_xprt.o
obj-$(CONFIG_MSM_RPC_PING) += ping_mdm_rpc_client.o
obj-$(CONFIG_MSM_RPC_PROC_COMM_TEST) += proc_comm_test.o
obj-$(CONFIG_MSM_RPC_PING) += ping_mdm_rpc_client.o ping_apps_server.o
obj-$(CONFIG_MSM_RPC_OEM_RAPI) += oem_rapi_client.o
obj-$(CONFIG_MSM_RPC_WATCHDOG) += rpc_dog_keepalive.o
obj-$(CONFIG_MSM_RPCSERVER_WATCHDOG) += rpc_server_dog_keepalive.o
obj-$(CONFIG_MSM_RPCSERVER_TIME_REMOTE) += rpc_server_time_remote.o
obj-$(CONFIG_MSM_DALRPC) += dal.o
obj-$(CONFIG_MSM_DALRPC_TEST) += dal_remotetest.o
obj-$(CONFIG_ARCH_MSM7X30) += dal_axi.o
obj-$(CONFIG_MSM_ADSP) += qdsp5/
obj-$(CONFIG_MSM7KV2_AUDIO) += qdsp5v2/
obj-$(CONFIG_MSM_RPCSERVER_HANDSET) += rpc_server_handset.o
obj-$(CONFIG_MSM_QDSP6) += qdsp6/
obj-$(CONFIG_MSM8X60_AUDIO) += qdsp6v2/
obj-$(CONFIG_MSM_HW3D) += hw3d.o
ifdef CONFIG_PM
	obj-$(CONFIG_ARCH_MSM8X60) += pm-8x60.o
	obj-$(CONFIG_ARCH_QSD8X50) += pm2.o
	obj-$(CONFIG_ARCH_MSM7X30) += pm2.o
	obj-$(CONFIG_ARCH_MSM7X27) += pm2.o
	obj-$(CONFIG_ARCH_MSM7X25) += pm.o
	obj-$(CONFIG_ARCH_MSM7X01A) += pm.o
else
	obj-y += no-pm.o
endif
obj-$(CONFIG_MSM_DMA_TEST) += dma_test.o
obj-$(CONFIG_SURF_FFA_GPIO_KEYPAD) += keypad-surf-ffa.o

obj-$(CONFIG_ARCH_MSM7X01A) += board-halibut.o devices-msm7x01a.o
obj-$(CONFIG_MACH_TROUT) += board-trout.o board-trout-gpio.o
obj-$(CONFIG_MACH_TROUT) += board-trout-keypad.o board-trout-panel.o
obj-$(CONFIG_MACH_TROUT) += htc_akm_cal.o htc_wifi_nvs.o htc_acoustic.o
obj-$(CONFIG_MACH_TROUT) += board-trout-mmc.o board-trout-wifi.o
obj-$(CONFIG_ARCH_QSD8X50) += devices-qsd8x50.o
obj-$(CONFIG_MACH_QSD8X50_SURF) += board-qsd8x50.o
obj-$(CONFIG_MACH_QSD8X50_FFA) += board-qsd8x50.o
#obj-$(CONFIG_MACH_QSD8X50_COMET) += board-comet.o
obj-$(CONFIG_MACH_QSD8X50A_SURF) += board-qsd8x50.o
obj-$(CONFIG_MACH_QSD8X50A_FFA) += board-qsd8x50.o
obj-$(CONFIG_MACH_QSD8X50A_ST1_5) += board-qsd8x50a-st1x.o
obj-$(CONFIG_ARCH_MSM8X60) += devices-msm8x60.o
obj-$(CONFIG_ARCH_MSM8X60) += clock-local.o clock-8x60.o acpuclock-8x60.o
obj-$(CONFIG_ARCH_MSM8X60) += clock-rpm.o spm.o restart.o
obj-$(CONFIG_ARCH_MSM8X60) += subsystem-fatal-8x60.o

ifdef CONFIG_ARCH_MSM8X60
	obj-$(CONFIG_CPU_IDLE) += cpuidle.o
endif

obj-$(CONFIG_MACH_MSM8X60_RUMI3) += board-msm8x60.o
obj-$(CONFIG_MACH_MSM8X60_SIM) += board-msm8x60.o
obj-$(CONFIG_MACH_MSM8X60_SURF) += board-msm8x60.o
obj-$(CONFIG_MACH_MSM8X60_FFA) += board-msm8x60.o
obj-$(CONFIG_TROUT_H2W) += board-trout-h2w.o
obj-$(CONFIG_TROUT_BATTCHG) += htc_battery.o
obj-$(CONFIG_TROUT_PWRSINK) += htc_pwrsink.o
obj-$(CONFIG_ARCH_MSM7X27) += board-msm7x27.o devices-msm7x27.o
obj-$(CONFIG_ARCH_MSM7X30) += board-msm7x30.o devices-msm7x30.o
obj-$(CONFIG_ARCH_MSM7X30) += clock-local.o clock-7x30.o acpuclock-7x30.o
obj-$(CONFIG_ARCH_MSM7X25) += board-msm7x27.o devices-msm7x25.o

obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire.o board-sapphire-gpio.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire-keypad.o board-sapphire-panel.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire-mmc.o board-sapphire-wifi.o
obj-$(CONFIG_MACH_SAPPHIRE) += board-sapphire-rfkill.o msm_vibrator.o

obj-$(CONFIG_TROUT_BATTCHG) += htc_battery.o

obj-$(CONFIG_HTC_PWRSINK) += htc_pwrsink.o
obj-$(CONFIG_HTC_HEADSET) += htc_headset.o
obj-$(CONFIG_MSM_RMT_STORAGE_CLIENT) += rmt_storage_client.o
obj-$(CONFIG_PMIC8058) += pmic8058-gpio.o pmic8058-mpp.o
obj-$(CONFIG_MSM_NPA) += npa.o npa_resources.o msm_pm_qos.o msm_reqs.o
obj-$(CONFIG_MSM_NPA_REMOTE) += npa_remote.o
obj-$(CONFIG_MSM_NPA_REMOTE_ONCRPC) += npa_remote_rpc_client.o
obj-$(CONFIG_ARCH_MSM7X30) += spm.o
obj-$(CONFIG_MSM_RPM) += rpm.o

obj-$(CONFIG_SMMU)		+= smmu_driver.o smmu_driver_enum.o
obj-$(CONFIG_SMMU_8X60)		+= board-msm8x60-smmu.o
ifdef CONFIG_GPIOLIB
obj-$(CONFIG_MSM7200A_TLMM)	+= msm7200a-gpio.o
else
obj-$(CONFIG_MSM7200A_TLMM)	+= gpio.o generic_gpio.o
endif
obj-$(CONFIG_MSM7200A_TLMM)	+= tlmm-msm7200a.o
obj-$(CONFIG_MSM8660_TLMM)	+= tlmm-msm8660.o msm8660-gpio.o
obj-$(CONFIG_MSM_BUS_SCALING)	+= msm_bus_core.o
obj-$(CONFIG_MSM7200A_TLMM)	+= gpiomux.o gpiomux-7x00.o
obj-$(CONFIG_MSM8660_TLMM)	+= gpiomux.o gpiomux-8x60.o
