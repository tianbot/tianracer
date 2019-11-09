#!/bin/bash

echo "Install systemd service and enable tianracer auto start"

sudo cp ./_service_/tianracer_bringup.service  /lib/systemd/system
sudo cp ./_bin_/tianracer_bringup.sh  /usr/local/bin
sudo chmod +x /usr/local/bin/tianracer_bringup.sh

echo " "
echo "Enable tianracer auto start"
echo ""
sudo systemctl daemon-reload
sudo systemctl enable tianracer_bringup.service

echo "finish. "
echo "Use 'systemctl disable tianracer_bringup.service' to disable auto start"

