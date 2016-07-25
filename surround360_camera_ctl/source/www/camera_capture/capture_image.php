<?php
        // Check if app is running
        exec("pgrep CameraControl", $output, $return);

        if ($return == 0) {
                echo 'ERROR:isAppRunning';
                return;
        }

        // TODO (app): allow dynamic number of cameras and frame rate
        $num_cameras = 17;
        $fps = 30;

        if (array_key_exists('checkbox_exposure', $_POST)) {
          $num_frames = 6; // Even number of frames guarantees two binary files of same size
        } else {
          $num_frames = (60 * $_POST['minutes'] + $_POST['seconds']) * $fps;
        }

        $label = $_POST['label'];

        // We do not want spaces on filenames
        $label = str_replace(' ', '_', $label);

        // Create all command line options
        $output_dir = time().'_'.$label;
        $_POST['dir'] = $output_dir;
        unset($_POST['label']);

        $cmd_options = ' -fps '.$fps.' -numcams '.$num_cameras.' -raw -nbits 8 -nframes '.$num_frames.' -dir '.$output_dir;

        // Camera properties
        $cmd_wb = array();
        foreach ($_POST as $cmd => $value) {
                // Ignore parameters that do not start with '-'
                if (strpos($cmd, '-') !== 0) {
                        continue;
                }
                if ($cmd === '-whitebalance1' || $cmd === '-whitebalance2') {
                        $cmd_wb[] = $value;
                } else {
                        $cmd_options .= sprintf(' %s %s', $cmd, $value);
                }
        }

        if (count($cmd_wb) > 0) {
                $cmd_options .= sprintf(' -whitebalance "%s %s"', $cmd_wb[0], $cmd_wb[1]);
        }

        // Free pagecache, dentries and inodes
        // TODO (app): run this after the process ends (too?)
        exec('sync; echo 3 | sudo tee /proc/sys/vm/drop_caches');

        $command = 'sudo /usr/local/bin/CameraControl '.$cmd_options;

        // Run first with -nocapture to set up all the cameras
        exec($command.' -nocapture', $output);
        sleep(1);
        $pid = exec('nohup '.$command.' > /dev/null 2>&1 & echo $!; ', $output);

        // Return PID and output directory
        echo json_encode(array($pid, $output_dir));
?>
