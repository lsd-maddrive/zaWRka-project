#/bin/bash

echo "Installing device rules"
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ydlidar"' | sudo tee /etc/udev/rules.d/ydlidar.rules
echo  'SUBSYSTEMS=="usb", KERNEL=="video*", ATTRS{idVendor}=="05a3", ATTRS{idProduct}=="9750", SYMLINK+="elp_stereo", MODE="0776"'  | sudo tee /etc/udev/rules.d/90-elp-stereo-camera.rules
echo  'SUBSYSTEMS=="usb", KERNEL=="video*", ATTRS{idVendor}=="0458", ATTRS{idProduct}=="708c", SYMLINK+="top_camera", MODE="0776"'  | sudo tee /etc/udev/rules.d/91-signs-camera.rules

sudo service udev reload
sleep 2
sudo service udev restart