<?php
/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl file in the root directory of this subproject.
*/

  $dir = $_GET['dir'];

  exec('pgrep CameraControl', $pids);
  if (empty($pids)) {
    echo 'NO';
    return;
  }

  // Process running. Read heartbeat (@ to suppress warnings)
  $fp = @fopen('/media/snoraid/'.$dir.'/heartbeat.dat', 'rb');

  $output_empty = 'No stats to report yet...';

  if ($fp === false) {
    echo $output_empty;
    return;
  }

  $fps = @unpack('i', fread($fp, 4));
  $secs = @unpack('i', fread($fp, 4));
  $dpf = @unpack('i', fread($fp, 4));

  if (!$fps || !$secs || !$dpf) {
    echo $output_empty;
    return;
  }

  echo 'Elapsed time: '.gmdate('i \m s \s', $secs[1]).', FPS: '.$fps[1] / 10.0.', Dropped frames so far: '.$dpf[1];

  return;
?>
