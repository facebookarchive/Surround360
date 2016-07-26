<?php
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
