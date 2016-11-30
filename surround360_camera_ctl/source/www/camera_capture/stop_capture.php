<?php
/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl file in the root directory of this subproject.
*/

  $pname = 'CameraControl';
  exec('pgrep ' . $pname, $pids);
  if (empty($pids)) {
    echo 'Process not yet running...';
    return;
  }

  // Process running. Kill it
  exec('sudo pkill -SIGINT ' . $pname);

  echo 'OK';
  return;
?>
