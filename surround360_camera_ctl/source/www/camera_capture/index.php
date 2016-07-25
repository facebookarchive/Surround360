<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8">

    <!-- Always force latest IE rendering engine or request Chrome Frame -->
    <meta content="IE=edge,chrome=1" http-equiv="X-UA-Compatible">
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />

    <!-- Use title if it's in the page YAML frontmatter -->
    <title>Surround 360</title>

    <!-- JS Libraries -->
    <script type="text/javascript" src="js/jquery-2.1.4.min.js"></script>
    <script type="text/javascript" src="js/jquery-ui.min.js"></script>
    <script type="text/javascript" src="js/slimbox2.js"></script>
    <script type="text/javascript" src="js/jquery.lazyload.js"></script>

    <link href="/css/all.css" rel="stylesheet" type="text/css" />
    <link href="/css/jquery-ui.css" rel="stylesheet" type="text/css" />
    <link rel="stylesheet" href="/css/slimbox/slimbox2.css" type="text/css" media="screen" />
  </head>

  <?php date_default_timezone_set('America/Los_Angeles'); ?>

  <body class="index">
    <div id="fb-root"></div>
    <div id="wrapper">
    <div class="hero">
      <div class="row">
        <div class="large-12 columns">
          <h3>
            <div>
              <img src="/img/fb_small_white.png" style="vertical-align:middle" />
              <span style="color:white">Surround 360</span>
            </div>
          </h3>
        </div>
      </div>
    </div>
    <div class="row">
      <div class="large-12 columns">
        <form id="capture_image" method="post" action="capture_image.php">
          <h4>Record Video</h4>
          &nbsp;
          <button id="button_snap" type="submit" class="btn btn-primary start">Start</button>
          <button id="button_stop" type="button" class="btn btn-primary stop" style="display:none">Stop</button>
          <button id="button_reset" type="button">Reset Params</button>
          <button id="button_update_preview" type="button">Update Preview</button>
          <div id="log_div"></div>
          <table id="table_label" border="0" cellspacing="0"><tr><td>Label:</td><td><input type="text" name="label" value="<?php echo time(); ?>" style="width: 14em;"></td></tr></table>
          <br>
          <div id="loading" style="display:none">
            <img src="/img/loading.gif" style="vertical-align:middle" />
            <span style="color:red" id="loading_text"></span>
            </br></br>
          </div>
          <table id="table_duration" border="0" cellspacing="0" id="cssTable">
            <tr>
              <td><input type="number" id="minutes"  name="minutes" min="0" max="60" step="1" value="0" style="width: 4em;"></td>
              <td>min</td>
              <td><input type="number" id="seconds" name="seconds" min="0" max="59" step="1" value="30" style="width: 4em;"></td>
              <td>sec</td>
            </tr>
          </table>
          </br>
          <div id="camera_control"></div>
        </form>
      </div>
    </div>
    <div class="row">
      <div class="large-12 columns">
        <h4>Preview</h4>
        <div id="preview">
          <select multiple="multiple" name="choose_cameras" id="choose_cameras" size="17">
            <option value="0">Camera 0</option>
            <option value="1">Camera 1</option>
            <option value="2">Camera 2</option>
            <option value="3">Camera 3</option>
            <option value="4">Camera 4</option>
            <option value="5">Camera 5</option>
            <option value="6">Camera 6</option>
            <option value="7">Camera 7</option>
            <option value="8">Camera 8</option>
            <option value="9">Camera 9</option>
            <option value="10">Camera 10</option>
            <option value="11">Camera 11</option>
            <option value="12">Camera 12</option>
            <option value="13">Camera 13</option>
            <option value="14">Camera 14</option>
            <option value="15">Camera 15</option>
            <option value="16">Camera 16</option>
          </select>
          <br>
          <video id="feed_preview" src="http://127.0.0.1:8090/live.webm" width="640" height="640" autoplay></video>
        </div>
      </div>
    </div>
    <div class="row">
      <div class="large-12 columns">
        <h4>Latest Video Stats</h4>
        <div id="log_stats_div"></div>
      </div>
    </div>

  <script type="text/javascript">
    var isCapturing = false;
    var isStopped = false;
    var pid = -1;
    var dirOutput = -1;
    var interval = -1;
    var defaultErrorText = "There was an error processing your request";

    function logger(msg) {
      document.getElementById('log_div').innerHTML = msg + "<br><br>";
    }

    function loggerStats(msg) {
      document.getElementById('log_stats_div').innerHTML = msg + "<br>";
    }

    $(document).ready(function () {
      interval = setInterval(checkIfPidRunning, 1000); // check every second

      $('#camera_control').load('camera_control.php');
      $('#button_reset').click(function(){
        // Reload camera controls
        $('#camera_control').load('camera_control.php');
      });

      $('#button_update_preview').click(function(){
        var shutter = document.getElementById('-shutter_number').value;
        var gain = document.getElementById('-gain_number').value;

        // Get list of cameras used for preview
        var cameras = $.map($('#choose_cameras option:selected'), function(el, i) {
          return $(el).val();
        });

        if (cameras.length < 4) {
          alert("Need to select 4 cameras");
          return;
        }

        $.ajax({
          url: 'preview_update.php?shutter=' + shutter + '&gain=' + gain + "&cameras=" + cameras.join(","),
          type: 'get',
          success: function(response) {
            if (response) {
              if (response == 'OK') {
                logger('Preview parameters updated...');
              } else {
                logger(response);
              }
            } else {
              alert("Update preview: bad response");
            }
          },
          error: function() {
            alert(defaultErrorText);
          }
        });
      });

      var last_valid_cam_selection = null;
      $('#choose_cameras').change(function(event) {
        if ($(this).val().length > 4) {
          $(this).val(last_valid_cam_selection);
        } else {
          last_valid_cam_selection = $(this).val();
        }
      });

      $('#button_stop').click(function(){
        isStopped = true;
        $.ajax({
          url: 'stop_pid.php?pid=' + pid,
          type: 'post',
          success: function(response) {
            if (response) {
              if (response == 'OK') {
                logger('Stopping process...');
              } else {
                logger(response);
              }
            } else {
              alert("Stop capture: bad response");
            }
          },
          error: function() {
            alert(defaultErrorText);
          }
        });
      });
    });
  </script>

  <script type="text/javascript">
    function showLoading() {
      $('#loading').show();
      $('#loading_text').text('');
      $('#button_snap').hide();
      $('#button_stop').show();
      $('#button_reset').hide();
      $('#table_label').hide();
      $('#table_duration').hide();
    }

    function clearLoading() {
      $('#loading').hide();
      $('#loading_text').text('');
      $('#button_snap').show();
      $('#button_stop').hide();
      $('#button_reset').show();
      $('#table_label').show()
      $('#table_duration').show();
    }

    function showLoadingBatch() {
      $('#loading_batch').show();
      $('#batch_html').text('');
      $('#button_apply_to_all').hide();
    }

    function clearLoadingBatch() {
      $('#loading_batch').hide();
      $('#button_apply_to_all').show();
    }

    function reloadAlbums() {
      $('#browse_pictures').load(location.href + " #browse_pictures", function() {
        // Reload lazyload
        $("img.lazy").lazyload();

        // Reload slimbox js for slideshow
        $.getScript('js/slimbox2.js')
      });
    }

    function freeCache() {
      $.ajax({
        url: 'drop_cache.php',
        type: 'post',
        success: function(response) {
          if (response) {
            if (response != 'OK') {
              logger(response);
            }
          } else {
            alert("Free cache: bad response");
          }
        },
        error: function() {
          alert(defaultErrorText);
        }
      });
    }

    function checkErrors() {
      var hasErrors = false;
      $.ajax({
        url: 'check_errors.php?dir=' + dirOutput,
        type: 'post',
        success: function(response) {
          if (response) {
            if (response != 'OK') {
              logger('ERROR: ' + response);
              hasErrors = true;
            }
          } else {
            alert("Check errors: bad response");
          }
        },
        error: function() {
          alert(defaultErrorText);
        }
      });
      return hasErrors;
    }

    function showStats() {
      $.ajax({
        url: 'show_stats.php?dir=' + dirOutput,
        type: 'post',
        success: function(response) {
          if (response) {
            if (response != 'OK') {
              loggerStats(response);
            }
          } else {
            alert("Show stats: bad response");
          }
        },
        error: function() {
          alert(defaultErrorText);
        }
      });
    }

    function hideLogger() {
      document.getElementById('log_div').innerHTML = '';
    }

    function doneCapturing() {
      isCapturing = false;
      isStopped = false;
      pid = -1;
      logger('Done capturing. Cleaning up...');

      // Free slab objects and pagecache
      freeCache();

      // Check if errors
      var hasErrors = checkErrors();

      //TODO (app): the above funcion makes an async ajax call, so it'll
      // always return false. Fix it!!

      if (hasErrors == true) {
        clearLoading();
      } else {
        showStats();

        // Clean logger and loading div
        // Giving it a 2 second pause to make sure everything is back in place
        setTimeout(function(){
          hideLogger();
          clearLoading();
        }, 2000);
      }
    }

    function checkIfPidRunning() {
      if (pid > 0 && isCapturing) {
        $.ajax({
          url: 'check_pid.php?pid=' + pid + '&dir=' + dirOutput,
          type: 'post',
          success: function(response) {
            if (response) {
              if (response == 'NO') {
                doneCapturing();
              } else if (!isStopped) {
                // Response will be heartbeat
                logger(response);
              }
            } else {
              alert("Check process: bad response");
            }
          },
          error: function() {
            alert(defaultErrorText);
          }
        });
      }
    }

    $(document).ready(function(){
      var form = $('#capture_image');
      $(form).submit(function(e){
        e.preventDefault();

        // Clear stats
        loggerStats('');

        showLoading();
        logger('Started capturing. Waiting for first heartbeat...');

        $.ajax({
          url: $(form).attr('action'),
          type: $(form).attr('method'),
          data: $(form).serialize(),
          success: function(response) {
            if (response) {
              if (response === 'ERROR:isAppRunning') {
                alert('Camera is busy. Please try again.');
                clearLoading();
              } else if (response === 'ERROR:isEverythingConnected') {
                $('#loading_text').text('Camera connection error. Reconnecting...');
                $.ajax({
                  url: 'reconnect_usb.php?n=' + $numDevices,
                  type: 'post',
                  success: function(response) {
                    alert(response);
                    clearLoading();
                  },
                  error: function() {
                    clearLoading();
                    alert("There was an error reconnecting the cameras");
                  }
                });
              } else {
                // Camera capture process is not done yet. It's in the background
                // and we return its PID
                var result = $.parseJSON(response);
                pid = parseInt(result[0]);
                dirOutput = String(result[1]);
                isCapturing = true;
              }
            } else {
              reloadAlbums();
              clearLoading();
            }
          },
          error: function() {
            clearLoading();
            alert(defaultErrorText);
          }
        });
      });

      var form_batch = $('#form_batch');
      $(form_batch).submit(function(e){
        e.preventDefault();
        var countCheckedCheckboxes = checkboxes.filter(':checked').length;
        var option = $("#apply_to_all option:selected").text();
        $('#dialog_batch').html('Applying ' + option + ' to ' + countCheckedCheckboxes + ' datasets. Are you sure?');
          $("#dialog_batch").dialog("open");
      });

      $("#dialog_batch").dialog({
         width: 500,
         autoOpen: false,
         modal: true,
         open: function() {
            $(".ui-widget-overlay").css({background: "#000", opacity: 0.9});
         },
         buttons : {
          "Confirm" : function() {
            $(this).dialog("close");
            showLoadingBatch();
              var form_batch = $('#form_batch');
              $.ajax({
              url: $(form_batch).attr('action'),
              type: $(form_batch).attr('method'),
              data: $(form_batch).serialize(),
              success: function(response) {
                //alert(response);
                $('#batch_html').html(response);
                clearLoadingBatch();

                if (response.indexOf("Deleted") > -1) {
                  reloadAlbums();
                }
              },
              error: function() {
                clearLoading();
                alert(defaultErrorText);
                clearLoadingBatch();
              }
            });
          },
          "Cancel" : function() {
            $(this).dialog("close");
          }
          }
        });

      var checkboxes = $("input:checkbox[class=chk]");
      $('#check-all').click(function(){
          checkboxes.prop('checked', true).trigger('change');
        });
      $('#uncheck-all').click(function(){
          checkboxes.prop('checked', false).trigger('change');
      });
      $('#check-invert').click(function(){
          checkboxes.each(function(){
            $(this).prop('checked', !$(this).prop('checked')).trigger('change');
          });
      });

      checkboxes.change(function(){
            var countCheckedCheckboxes = checkboxes.filter(':checked').length;
            $('#count-checked').text(countCheckedCheckboxes);
        });
    });
  </script>
  </body>
</html>
