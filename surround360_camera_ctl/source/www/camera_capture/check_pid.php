<?php
  $pid = $_GET['pid'];
  $dir = $_GET['dir'];
  
  if (!file_exists("/proc/$pid")) {
    echo 'NO';
    return;
  }
  
  // Process running. Read heartbeat (@ to suppress warnings)
  $fp = @fopen('/var/log/vrcam/captures/'.$dir.'/heartbeat.bin', 'rb');
  
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
