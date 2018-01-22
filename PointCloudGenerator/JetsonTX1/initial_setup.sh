# Paste this into an SSH console to perform first-time setup of the Jetson.
# Assumes:
#  - user interactivity
#  - An SD card with:
#    * The latest flycapture drivers https://www.ptgrey.com/support/downloads/10703/
#    * .deb files for Lynx

# Fix hostname
HN=optical-guide0
sudo sed -i s/tegra-ubuntu/$HN/g /etc/hosts
sudo sed -i s/tegra-ubuntu/$HN/g /etc/hostname
sudo hostname $HN

# Mount SD card
sudo mkdir /media/sd_card
sudo mount /dev/mmcblk1p1 /media/sd_card 
cd /media/sd_card

# Install lynx
sudo dpkg --install lynx_2.8.9dev8-4ubuntu1_arm64.deb lynx-common_2.8.9dev8-4ubuntu1_all.deb
lynx google.com # requires creare credentials

# Update packages
sudo apt-get -y update
sudo apt-get -y upgrade
sudo apt-get -y install git nano tree dos2unix device-tree-compiler cmake libqt4-dev

# Add passwordless sudo (after the update, since the update affects sudoers)
sudo bash -c 'echo "ubuntu ALL= NOPASSWD: ALL" | (EDITOR="tee -a" visudo)'

# Checkout repo and set up hooks
cd /media/sd_card
git clone --bare https://code.crearecomputing.com/UAV/OpticalGuide.git
cd OpticalGuide.git
git show HEAD:PointCloudGenerator/JetsonTX1/hooks/post-receive > hooks/post-receive

# Replace with UTC timestamp - MMDDhhmmCCYY
sudo date 033115102017

# Hook configures /etc/
cd /media/sd_card/OpticalGuide.git
echo nothing | ./hooks/post-receive

# Install Auvidea's support for SPI peripherals
cd /media/sd_card/OpticalGuide/PointCloudGenerator/JetsonTX1/
bash install_spi_support.sh

# Install flycapture
cd /media/sd_card/OpticalGuide/PointCloudGenerator/JetsonTX1/
bash install_flycap.sh
