<?php
/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl file in the root directory of this subproject.
*/

  $action = $_GET['action'];
  if ($action != "record" && $action != "stop" && $action != "quit") {
    echo "ERROR";
    return;
  }
  $key = ftok("/usr/local/bin/CameraControl", 'S');
  $queue = msg_get_queue($key, 0666);
  $action_cmd_msg = $action . "\0";

  $ret = msg_send($queue, 1, $action_cmd_msg, false, true, $msg_err);
  if (!$ret) {
    echo "ERROR";
  } else {
    echo "OK";
  }
?>
