<?php
        $shutter = $_GET['shutter'];
        $gain = $_GET['gain'];
        $cameras = $_GET['cameras'];
        $key = ftok("/usr/local/bin/CameraControl", 'S');
        $queue = msg_get_queue($key, 0666);
        $shutter_cmd_msg = "shutter " . $shutter . "\0";
        $gain_cmd_msg = "gain " . $gain . "\0";

        $ret1 = msg_send($queue, 1, $shutter_cmd_msg, false, true, $msg_err);
        $ret2 = msg_send($queue, 1, $gain_cmd_msg, false, true, $msg_err);

        $tokens = explode(",", $cameras);

        $i = 0;
        foreach ($tokens as $token) {
            $cmd = "cam " . $i . " " . $token . "\0";
            $ret3 = msg_send($queue, 1, $cmd, false, true, $msg_err);
            if (!$ret3) {
               echo "ERROR";
               return;
            }
            $i = $i + 1;
        }

        if (!($ret1 && $ret2 && $ret3)) {
            echo "ERROR";
        } else {
            echo "OK";
        }
?>
