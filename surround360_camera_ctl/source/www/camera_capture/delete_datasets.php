<?php
	if (!isset($datasets_delete) || empty($datasets_delete)) {
		$datasets_delete = $_GET["datasets"];
	}

	$n = 0;
	foreach ($datasets_delete as $dataset) {
		$dir_dataset = realpath('.\\captures\\'.$dataset);
		exec('rmdir "'.$dir_dataset.'" /s /q'); 
		$n++;
	}

	echo 'Deleted '.$n.' datasets';
?>
