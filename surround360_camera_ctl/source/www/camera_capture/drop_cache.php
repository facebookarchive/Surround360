<?php
  $result = exec('sudo /bin/bash /usr/local/bin/drop_cache.sh', $output, $return_var);
  echo 'OK';
  return;
?>
