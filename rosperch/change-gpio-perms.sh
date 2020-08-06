#!/bin/bash
sudo groupadd gpio
sudo chown ubuntu:gpio /dev/mem
sudo chown ubuntu:gpio /dev/gpiomem
sudo chmod g+rw /dev/mem
sudo chmod g+rw /dev/gpiomem
sudo usermod -aG gpio ubuntu
