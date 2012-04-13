#!/bin/bash

rmmod temperature_driver
sleep 1
rm /dev/temperature

sleep 1
mknod /dev/temperature c 74 0
insmod temperature_driver.o

dmesg


