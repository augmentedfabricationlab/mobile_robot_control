#!/bin/bash
echo "removing mozilla history..."
rm ~/.mozilla ~/.cache/mozilla -rfv
echo "removing Download and Desktop folders..."
rm ~/Downloads/* && rm ~/Deksktop/* -rfv
echo "removing git config..."
rm ~/.gitconfig -v
echo "Removing ROS logs"
rm -rf $HOME/.ros/log/*
echo "removing bash history..."
cat /dev/null > ~/.bash_history
rm -rf $HOME/.vscode-server
find /var/log -type f -exec sudo rm {} \;
