#!/bin/bash

sudo route add -host 192.168.0.3 dev enp2s2
sudo route add -host 192.168.0.200 dev eno1

sudo route add -host 192.168.0.210 dev eno1



