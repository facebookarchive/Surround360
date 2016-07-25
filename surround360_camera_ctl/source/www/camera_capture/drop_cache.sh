#!/bin/bash
/bin/sync; /bin/echo 3 | /usr/bin/tee /proc/sys/vm/drop_caches
