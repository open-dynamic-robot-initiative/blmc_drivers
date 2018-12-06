#! /bin/bash

sudo ip link set can0 type can bitrate 1000000
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can2 type can bitrate 1000000
sudo ip link set can3 type can bitrate 1000000
sudo ip link set can4 type can bitrate 1000000
sudo ip link set can5 type can bitrate 1000000
sudo ip link set can6 type can bitrate 1000000
sudo ip link set can7 type can bitrate 1000000

sudo ip link set up can0
sudo ip link set up can1
sudo ip link set up can2
sudo ip link set up can3
sudo ip link set up can4
sudo ip link set up can5
sudo ip link set up can6
sudo ip link set up can7

ip link show

netstat -i
