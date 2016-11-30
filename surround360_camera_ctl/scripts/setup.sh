# Copyright (c) 2016-present, Facebook, Inc.
# All rights reserved.
#
# This source code is licensed under the license found in the
# LICENSE_camera_ctl file in the root directory of this subproject.

#!/bin/bash

mount -o noatime /dev/sda /media/snoraid
echo noop > /sys/block/sda/queue/scheduler
echo 0 > /sys/block/sda/queue/rotational
echo 1024 > /sys/block/sda/queue/nr_requests
for CPUFREQ in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do [ -f $CPUFREQ ] || continue; echo -n performance > $CPUFREQ; done
