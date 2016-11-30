<?php
/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl file in the root directory of this subproject.
*/

  $previewSlot = $_GET['slot'];
  $previewCam = $_GET['camera'];

  $key = ftok("/usr/local/bin/CameraControl", 'S');
  $queue = msg_get_queue($key, 0666 | IPC_CREAT);

  $cmd_msg = "cam " . $previewSlot . " " . $previewCam . "\0";
  $ret = msg_send($queue, 1, $cmd_msg, false, true, $msg_err);
  if (!$ret) {
    echo "Error. Msg not sent";
  }
?>
