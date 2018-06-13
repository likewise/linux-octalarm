# Overlays for PHYTEC RPi HAT support

dtbo-y += \
	imx7-peb-d-sense.dtb \
	imx7-peb-d-unicornhd.dtb

targets += dtbs dtbs_install
targets += $(dtbo-y)

always		:= $(dtbo-y)
clean-files	:= *.dtb

DTC_FLAGS ?= -@ -H epapr
