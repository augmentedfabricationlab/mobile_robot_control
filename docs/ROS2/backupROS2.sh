#!/bin/bash

ROBOT_ID=robot

# Create folders
mkdir -p ~/backup_$(hostname)/home/$ROBOT_ID/robot_ws
mkdir -p ~/backup_$(hostname)/home/$ROBOT_ID/deploy
mkdir -p ~/backup_$(hostname)/etc/udev/
mkdir -p ~/backup_$(hostname)/etc/systemd/user/

# Copy data to temporary folder
# From home
cp -rv ~/.bashrc ~/backup_$(hostname)/home/$ROBOT_ID/
cp -r ~/robot_ws/src ~/backup_$(hostname)/home/$ROBOT_ID/robot_ws
cp -r ~/deploy ~/backup_$(hostname)/home/$ROBOT_ID
# Files and config from system
sudo cp -rv /etc/udev/rules.d ~/backup_$(hostname)/etc/udev/

# Compress and delete temporary folder
sudo tar -C ~/ -czf ~/backup_$(hostname).tar.gz backup_$(hostname)
sudo rm -r ~/backup_$(hostname)
