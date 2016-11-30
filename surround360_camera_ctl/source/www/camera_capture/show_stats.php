<?php
  date_default_timezone_set('America/Los_Angeles');

  $dir = $_GET['dir'];

  $path = '/media/snoraid/'.$dir;
  $path_stats = $path.'/stats.txt';
  $path_dropped = $path.'/dropped_frames.txt';

  $time_modified = @filemtime($path_stats);
  if ($time_modified == false) {
    echo 'No stats available';
    return;
  }

  echo date('M j G:i:s T', $time_modified).' ('.$dir.')';

  echo '<br><br>';
  echo '<pre>';

  // Get stats
  system('/bin/cat '.$path_stats);

  echo '<br><br>';

  // Get dropped frames counts
  system('/bin/cat '.$path_dropped);

  echo '</pre>';

  return;
?>
