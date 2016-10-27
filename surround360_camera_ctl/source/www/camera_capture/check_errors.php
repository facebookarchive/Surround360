<?php
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
