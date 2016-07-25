<?php
	if (!isset($datasets_process) || empty($datasets_process)) {
		$datasets_process = $_GET["datasets"];
	}

	$db_captures = 'C:\Users\svc-vrcam\Dropbox (Facebook)\Apps\vrcam\captures_temp';

	$n = 0;
	foreach ($datasets_process as $dataset) {
		$dir_dataset = realpath('.\\captures\\'.$dataset);
		
		// Copy images (ignore small_*) to Dropbox "captures_temp" for processing
		// NOTE: "captures_temp" is automatically scanned and processed in the MacPro
		exec('robocopy "'.$dir_dataset.'" "'.$db_captures.'/'.$dataset.'" /E /XC /XN /XO /XF "small_*" "cubemap_*" "cylImg*" "pano_cmd.txt"');

		$n++;
	}

	echo 'Added '.$n.' datasets to the queue';
?>
