#!/bin/bash

echo "remap the device serial port(ttyUSBX) to rplidar and racecar"
echo "check it using the command : ls -l /dev|grep ttyUSB"

sudo rm /etc/udev/rules.d/99-tianbot*
sudo cp ./_udev_/*.rules  /etc/udev/rules.d

echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart

echo "finish "
echo "BY Maxwell AT:2018.07.07"
