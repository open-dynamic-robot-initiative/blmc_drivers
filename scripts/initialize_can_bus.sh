#! /bin/bash

sudo ip link set down can0
sudo ip link set down can1
sudo ip link set down can2
sudo ip link set down can3
sudo ip link set down can4
sudo ip link set down can5
sudo ip link set down can6
sudo ip link set down can7

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

ifconfig can0 txqueuelen 1000
ifconfig can1 txqueuelen 1000
ifconfig can2 txqueuelen 1000
ifconfig can3 txqueuelen 1000
ifconfig can4 txqueuelen 1000
ifconfig can5 txqueuelen 1000
ifconfig can6 txqueuelen 1000
ifconfig can7 txqueuelen 1000

ip link show

netstat -i
