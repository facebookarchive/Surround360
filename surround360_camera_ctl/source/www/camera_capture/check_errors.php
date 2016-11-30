<?php
/**
* Copyright (c) 2016-present, Facebook, Inc.
* All rights reserved.
*
* This source code is licensed under the license found in the
* LICENSE_camera_ctl file in the root directory of this subproject.
*/

  $dir = $_GET['dir'];

  $error_log = '/media/snoraid/captures/'.$dir.'/error.log';
  $fp = @fopen($error_log, 'rb');

  if ($fp == false) {
    echo 'OK';
    return;
  }

  system('cat '.$error_log);

  return;
?>
