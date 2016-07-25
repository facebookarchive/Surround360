<?php
	$root = '.\\captures\\';
	$albums = array();
	$album_processed_times = array();
	$have_cubemap = array();

	$db_captures = 'C:\Users\svc-vrcam\Dropbox (Facebook)\Apps\vrcam\captures_temp\\';
	
	foreach(preg_grep('/^([^.])/', scandir($root, SCANDIR_SORT_DESCENDING)) as $filename) {
		// Keep only last 50 datasets locally
		// TODO (app): let syncing script take care of this
		/* if (count($albums) == 50 && is_dir($root.$filename)) {
			exec('rmdir '.$root.$filename.' /s /q');
			continue;
		} */
		
		if ($filename == '.' || $filename == '..') {
			continue;
		}

		// Check if dataset has been processed, and move new files to local storage
		// NOTE: at this point the MacPro has already backed up all the files to Dropbox (captures)
		$path_local = $root.$filename;
		$path_dropbox = $db_captures.$filename;

		if (is_dir($path_dropbox) && file_exists($path_dropbox.'\\cubemap_'.$filename.'.png')) {
			exec('robocopy "'.$path_dropbox.'" "'.$path_local.'" /E /MOVE');
			exec('rmdir "'.$path_dropbox.'" /s /q');
		}

		$albums[$filename] = preg_grep('/^([^.])/', scandir($path_local));

		// Check if the album has cubemap
		$have_cubemap[$filename] = false;
		$path_cubemap = $path_local.'\\cubemap_'.$filename.'.png';

		$album_processed_times[$filename] = filemtime($path_local);
		if (file_exists($path_cubemap)) {
			$have_cubemap[$filename] = true;
			$album_processed_times[$filename] = filemtime($path_cubemap);
		}
	}
	
	// Sort by last processed time (descending)
	arsort($album_processed_times);
	
	foreach ($album_processed_times as $album => $time_modified) {
		$images = $albums[$album];
		$album_html = '';
		$is_raw = false;
		$has_cubemap = $have_cubemap[$album];

		if ($have_cubemap[$album]) {
			$is_raw = true;

			// Show only left pano
			$img_left = $root.$album.'/cylImgL_'.$album.'.png';
			$img_left_small = $root.$album.'/small_cylImgL_'.$album.'.png';

			if (!file_exists($img_left_small)) {
				$album_html .= 'Not found or corrputed';
			} else {
				$last_processed = date ('D M j Y, h:i:s A T', filemtime($img_left_small));
				$album_html .= '</br></br>';
				$album_html .= '<a href="'.$img_left_small.'" rel="lightbox-'.$album.'" title="<a href=&quot;'.$img_left.'&quot; target=&quot;_blank&quot; >Full-size</a>" altitle="">';
				$album_html .= '<img class="lazy" data-original="'.$img_left_small.'" width="1500" />';
				$album_html .= '</a>';
			}
		} else {
			foreach($images as $image) {
				// Ignore images with prefix "small_" and log/txt files
				$extension = pathinfo($image)['extension'];
				if (strpos($image, 'small_') !== false || $extension === 'log' || $extension === 'txt') {
					continue;
				}
				if (!$is_raw && strpos($image, '_raw') !== false) {
					$is_raw = true;
				}
				$img_src = $root.$album.'/'.$image;
				$img_src_small = $root.$album.'/small_'.$image;
				$album_html .= '<a href="'.$img_src_small.'" rel="lightbox-'.$album.'" title="<a href=&quot;'.$img_src.'&quot; target=&quot;_blank&quot; >Full-size</a>" altitle="">';
				$album_html .= '<img class="lazy" data-original="'.$img_src_small.'" width="150" />';
				$album_html .= '</a>';
			}
		}
		
		if ($album_html !== '') {
			echo '<input type="checkbox" class="chk" name="checkbox_dataset[]" value="'.$album.'" style="zoom:1.5">&nbsp;';
			$date_link = date('D M j Y, h:i:s A T', $album);
			$album_key = '/captures/'.$album;
			$date_link = '<a href="'.$root.$album.'" target="_blank" title="Dropbox">'.$date_link.'</a>';

			echo $date_link;
			if ($is_raw) {
				echo '&nbsp;&nbsp;&nbsp;';
				if (file_exists($db_captures.$album)) {
					echo 'Already queued';
				} else {
					$text = ($have_cubemap[$album] ? 'Re-' : '').'Add to processing queue';
					echo '<button id="button_process" type="button" onclick="addToProcessingQueue(\''.$album.'\')">'.$text.'</button>';
				}
			}
			if ($have_cubemap[$album]) {
				echo '&nbsp;&nbsp;&nbsp;<a href="'.$root.$album.'/cubemap_'.$album.'.png'.'" target="_blank" title="cubemap">Cubemap</a>';
			}
			echo '&nbsp;&nbsp;&nbsp;<button id="button_delete_dataset" type="button" onclick="deleteDataset(\''.$album.'\')">Delete dataset</button>';
			
			if ($has_cubemap) {
				echo '</br>Last processed: '.$last_processed;
			}
			echo '<div class="thumbnails">';
			echo $album_html;
			echo '</div>';
			echo '</br></br>';
		}
	}
	
	// When pressing button to process dataset, move directory to captures_temp
	echo '<script type="text/javascript">';
	echo '	function addToProcessingQueue(path) {';
	echo '		$.ajax({';
	echo '	  	type : "POST",';
	echo '	  	url: "enqueue_datasets.php?datasets[]=" + path,';
	echo '	  	success: function(response) {';
	echo '			alert(response);';
	echo '		}';
	echo '	});';
	echo '	}';
	echo '</script>';

	// When pressing button to delete dataset, remove directory from local storage
	echo '<script type="text/javascript">';
	echo '	function deleteDataset(path) {';
	echo '		$.ajax({';
	echo '	  	type : "POST",';
	echo '	  	url: "delete_datasets.php?datasets[]=" + path,';
	echo '	  	success: function(response) {';
	echo '			alert(response);';

	echo '			$("#browse_pictures").load(location.href + " #browse_pictures", function() {';
	echo '				$("img.lazy").lazyload();';
	echo '				$.getScript("js/slimbox2.js")';
	echo '			});';

	echo '		}';
	echo '	});';
	echo '	}';
	echo '</script>';
	
	// This will let us load images as we scroll down
	echo '<script type="text/javascript" charset="utf-8">';
	echo '	$(function() {';
	echo '		$("img.lazy").lazyload();';
	echo '	});';
	echo '</script>';
?>
