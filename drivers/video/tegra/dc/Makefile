GCOV_PROFILE := y
EXTRA_CFLAGS += -Idrivers/video/tegra/host
obj-y += dc.o bandwidth.o mode.o clock.o lut.o csc.o window.o
obj-y += rgb.o
obj-y += of_dc.o
obj-y += hdmi.o
ifndef CONFIG_MACH_BOWSER
obj-y += dsi2lvds.o
obj-y += tc358770_dsi2edp.o
else
obj-y += bowser-dsi2lvds.o
obj-y += tc358767_dsi2edp.o
endif
obj-$(CONFIG_TEGRA_MIPI_CAL) += mipi_cal.o
obj-$(CONFIG_TEGRA_NVHDCP) += nvhdcp.o
obj-y += edid.o
obj-y += nvsd.o
obj-y += dsi.o
obj-y += dc_sysfs.o
obj-y += dc_config.o
obj-$(CONFIG_TEGRA_DC_EXTENSIONS) += ext/
