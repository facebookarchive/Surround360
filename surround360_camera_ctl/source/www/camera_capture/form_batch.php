<?php
	if (!isset($_POST['checkbox_dataset']) || empty($_POST['checkbox_dataset']) || count($_POST['checkbox_dataset']) == 0) {
		echo 'No datasets selected';
	} else {

		$root = __DIR__.'\\captures\\';

		if ($_POST['apply_to_all'] == 'cubemaps') {
			$dir_cubemaps = '.\\cubemaps\\'.time();
			mkdir($dir_cubemaps);
		}
		

		foreach ($_POST['checkbox_dataset'] as $dataset) {
			$path_local = $root.$dataset;

			if ($_POST['apply_to_all'] == 'cubemaps') {
				// Create symlinks
				$cubemap_file = '\\cubemap_'.$dataset.'.png';
				symlink($path_local.$cubemap_file, $dir_cubemaps.$cubemap_file);
			}

			if ($_POST['apply_to_all'] == 'process') {
				$datasets_process[] = $dataset;
			}

			if ($_POST['apply_to_all'] == 'delete') {
				$datasets_delete[] = $dataset;
			}
		}

		if ($_POST['apply_to_all'] == 'cubemaps') {
			echo '<a href="'.$dir_cubemaps.'" target="_blank" title="cubemaps">Get cubemaps!</a>';
		}
		
		if ($_POST['apply_to_all'] == 'process') {
			// Var $datasets_process is used in `enqueue_datasets`, so we just have to include the file
			include 'enqueue_datasets.php';
		}

		if ($_POST['apply_to_all'] == 'delete') {
			// Var $datasets_delete is used in `delete_datasets`, so we just have to include the file
			include 'delete_datasets.php';
		}
	}
?>
