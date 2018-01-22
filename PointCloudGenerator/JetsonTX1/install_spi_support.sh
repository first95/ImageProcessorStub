#!/bin/bash
# Most of this is courtesy http://elinux.org/Jetson/TX1_SPI
cd ~
# Download and install Jetson kernel module "spidev"
wget --no-check-certificate 'https://developer.nvidia.com/embedded/dlc/l4t-sources-24-2-1' -O sources_r24.2.1.tbz2
# wget --no-check-certificate 'http://developer.nvidia.com/embedded/dlc/l4t-sources-27-1' -O sources_r27.2.1.tbz2
tar -xf sources_r24.2.1.tbz2
cd sources
tar -xf kernel_src.tbz2
cd kernel
zcat /proc/config.gz > .config
sed -i 's/# CONFIG_SPI_SPIDEV is not set/CONFIG_SPI_SPIDEV=m/' .config
sed -i 's/CONFIG_LOCALVERSION=""/CONFIG_LOCALVERSION="-tegra"/' .config
make prepare
make modules_prepare
make M=drivers/spi/
sudo cp drivers/spi/spidev.ko /lib/modules/$(uname -r)/kernel/drivers
sudo depmod

# Now compile and install the device tree.
cd /media/sd_card/OpticalGuide/PointCloudGenerator/JetsonTX1/boot/
NEW_DTB=auvidea_merged_with_tegra210-jetson-tx1.dtb
OLD_DTB=tegra210-jetson-tx1-p2597-2180-a01-devkit.dtb
DTC_CMD=dtc # You can also use ~/sources/kernel/scripts/dtc/dtc
$DTC_CMD -I dts -O dtb -o $NEW_DTB auvidea_merged_with_tegra210-jetson-tx1.dts

# Install
sudo cp $NEW_DTB /boot/
sudo sed -i s/$OLD_DTB/$NEW_DTB/g /boot/extlinux/extlinux.conf

# Compile NVIDIA's loopback test
cd ~/sources/kernel/Documentation/spi
gcc -o spidev_test spidev_test.c
sudo ./spidev_test -D /dev/spidev1.0
