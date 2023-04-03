#!/bin/sh
chmod 777 /sys/kernel/debug

chmod 666 /sys/kernel/debug/bluetooth/hci0/conn_min_interval
chmod 666 /sys/kernel/debug/bluetooth/hci0/conn_max_interval
chmod 666 /sys/kernel/debug/bluetooth/hci0/conn_info_max_age
chmod 666 /sys/kernel/debug/bluetooth/hci0/adv_min_interval
chmod 666 /sys/kernel/debug/bluetooth/hci0/adv_max_interval
chmod 666 /sys/kernel/debug/bluetooth/hci0/supervision_timeout

echo 6 > /sys/kernel/debug/bluetooth/hci0/conn_min_interval 
echo 8 > /sys/kernel/debug/bluetooth/hci0/conn_max_interval
echo 65535 > /sys/kernel/debug/bluetooth/hci0/conn_info_max_age 
echo 160 > /sys/kernel/debug/bluetooth/hci0/adv_min_interval 
echo 160 > /sys/kernel/debug/bluetooth/hci0/adv_max_interval
echo 3200 > /sys/kernel/debug/bluetooth/hci0/supervision_timeout