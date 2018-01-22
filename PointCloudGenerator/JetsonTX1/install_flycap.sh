mkdir ~/Downloads
ln -s /media/sd_card ~/creareData
cp ~/creareData/flycapture.2.10.3.266_arm64.tar.gz ~/Downloads

sudo mv /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.bak
sudo cp extlinux.conf /boot/extlinux/extlinux.conf

sudo apt-get -y install libraw1394-11 libgtkmm-2.4-1v5 libglademm-2.4-1v5 libusb-1.0-0
sudo apt-get -y install build-essential

cd ~/Downloads
tar xvfz flycapture.2.10.3.266_arm64.tar.gz
cd flycapture.2.10.3.266_arm64/lib
sudo cp libflycapture* /usr/lib
sudo cp C/libfly* /usr/lib
cd ..
sudo sh flycap2-conf

echo "PLEASE reboot system for changes to take effect"
