#!/bin/bash
# Decription:
# This is a shell file that adds the user
# ubuntu to the GPIO group, thus granting you
# access to it and allowing you to run code
# with GPIO pins without sudo privileges
# License:
# Software License Agreement (BSD License)
# Find the full agreement at https://github.com/amichael1227/ROSPerch/blob/master/LICENSE

sudo groupadd gpio
sudo chown ubuntu:gpio /dev/mem
sudo chown ubuntu:gpio /dev/gpiomem
sudo chmod g+rw /dev/mem
sudo chmod g+rw /dev/gpiomem
sudo usermod -aG gpio ubuntu