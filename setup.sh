#!/bin/bash
sudo apt-get install libmodbus-dev -y

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="zlac_motor"' >/etc/udev/rules.d/zlac_motor.rules

service udev reload
sleep 2
service udev restart