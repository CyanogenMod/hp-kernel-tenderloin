power_supply-objs := power_supply_core.o

ifeq ($(CONFIG_SYSFS),y)
power_supply-objs += power_supply_sysfs.o
endif

ifeq ($(CONFIG_LEDS_TRIGGERS),y)
power_supply-objs += power_supply_leds.o
endif

ifeq ($(CONFIG_POWER_SUPPLY_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif

obj-$(CONFIG_POWER_SUPPLY)	+= power_supply.o

obj-$(CONFIG_PDA_POWER)		+= pda_power.o
obj-$(CONFIG_APM_POWER)		+= apm_power.o
obj-$(CONFIG_MAX8925_POWER)	+= max8925_power.o
obj-$(CONFIG_WM831X_BACKUP)	+= wm831x_backup.o
obj-$(CONFIG_WM831X_POWER)	+= wm831x_power.o
obj-$(CONFIG_WM8350_POWER)	+= wm8350_power.o
obj-$(CONFIG_TEST_POWER)	+= test_power.o

obj-$(CONFIG_BATTERY_DS2760)	+= ds2760_battery.o
obj-$(CONFIG_BATTERY_DS2782)	+= ds2782_battery.o
obj-$(CONFIG_BATTERY_PMU)	+= pmu_battery.o
obj-$(CONFIG_BATTERY_OLPC)	+= olpc_battery.o
obj-$(CONFIG_BATTERY_TOSA)	+= tosa_battery.o
obj-$(CONFIG_BATTERY_COLLIE)	+= collie_battery.o
obj-$(CONFIG_BATTERY_WM97XX)	+= wm97xx_battery.o
obj-$(CONFIG_BATTERY_BQ27x00)	+= bq27x00_battery.o
obj-$(CONFIG_BATTERY_DA9030)	+= da9030_battery.o
obj-$(CONFIG_BATTERY_MAX17040)	+= max17040_battery.o
obj-$(CONFIG_BATTERY_Z2)	+= z2_battery.o
obj-$(CONFIG_CHARGER_PCF50633)	+= pcf50633-charger.o
obj-$(CONFIG_BATTERY_MSM)	+= msm_battery.o
obj-$(CONFIG_BATTERY_MSM8X60)	+= msm_charger.o
obj-$(CONFIG_PM8058_CHARGER)	+= pmic8058-charger.o
obj-$(CONFIG_ISL9519_CHARGER)	+= isl9519q.o
obj-$(CONFIG_PM8058_FIX_USB)	+= pm8058_usb_fix.o
obj-$(CONFIG_MAX8903B_CHARGER)	+= max8903b_charger.o
obj-$(CONFIG_PM8058_USB_VBUS_CONTROL)	+= pm8058_usb_vbus.o
obj-$(CONFIG_BATTERY_QCIBAT)	+= qci_battery.o
obj-$(CONFIG_SMB137B_CHARGER)   += smb137b.o
