<?php
  $pid = $_GET['pid'];
  
  if (!file_exists("/proc/$pid")) {
    echo 'Process not yet running...';
    return;
  }
  
  // Process running. Kill it
  $result = exec('sudo pkill -SIGINT CameraControl', $output, $return_var);
  
  echo 'OK';
  return;
?>
