#!/bin/bash

mount -o noatime /dev/sda /media/snoraid
echo noop > /sys/block/sda/queue/scheduler
echo 0 > /sys/block/sda/queue/rotational
echo 1024 > /sys/block/sda/queue/nr_requests
for CPUFREQ in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do [ -f $CPUFREQ ] || continue; echo -n performance > $CPUFREQ; done
