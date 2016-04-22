#!/bin/bash

cd $WORKSPACE
source /opt/fsl-imx-release/1.8/environment-setup-cortexa7hf-vfp-neon-poky-linux-gnueabi
make distclean
make imx7_phyboard_zeta_defconfig
make zImage
make imx7-phyboard-zeta.dtb
mkdir images
rm -rf images/*
cd arch/arm/boot/
tar -cvjf "${WORKSPACE}/images/linux-b${BUILD_ID}.tar.bz2" zImage dts/imx7-phyboard-zeta.dtb
