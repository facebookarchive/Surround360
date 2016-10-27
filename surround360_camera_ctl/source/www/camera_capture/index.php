<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <title>Surround 360</title>

    <script type="text/javascript" src="js/jquery-2.1.4.min.js"></script>
    <script type="text/javascript" src="js/jquery-ui.min.js"></script>

    <link href="/css/customized-ui.css" rel="stylesheet" type="text/css" />
    <link href="/css/jquery-ui.css" rel="stylesheet" type="text/css" />
  </head>

  <?php date_default_timezone_set('America/Los_Angeles'); ?>

  <body class="index">
    <div id="wrapper">
    <div class="hero">
      <div class="row">
        <div class="columns">
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
      <div class="columns">
        <form id="form_capture" method="post" action="capture_image.php">
          <h4>Record Video</h4>
          <button id="button_start_preview" type="submit">Start preview</button>
          <button id="button_stop_preview" type="button" style="display:none">Stop preview</button>
          <button id="button_start_record" type="button" style="display:none">Start recording</button>
          <button id="button_stop_record" type="button" style="display:none">Stop recording</button>
          <button id="button_reset" type="button">Reset Params</button>
          <button id="button_update_preview" type="button" style="display:none">Update Preview</button>
          <div id="log_div"></div>
          <table id="table_label" border="0" cellspacing="0">
            <tr>
              <td>Label:</td>
              <td><input type="text" name="label" value="<?php echo time(); ?>" style="width: 14em;"></td>
            </tr>
          </table>
          <div id="loading" style="display:none">
            <img src="/img/loading.gif" style="vertical-align:middle" />
            <span style="color:red" id="loading_text"></span>
            <br><br>
          </div>
          <br>
          <div id="camera_control"></div>
        </form>
      </div>
    </div>
    <div class="row">
      <div class="columns">
        <h4>Preview</h4>
        <div id="preview">
          <p>Choose 4 cameras:</p>
            <table id="table_checkbox" border="0" cellspacing="0">
              <tr>
                <td><input class="camera-checkbox" type="checkbox" name="choose_cameras" value="0">&nbsp;0&nbsp;</td>
                <td><input class="camera-checkbox" type="checkbox" name="choose_cameras" value="1">&nbsp;1&nbsp;</td>
                <td><input class="camera-checkbox" type="checkbox" name="choose_cameras" value="2">&nbsp;2&nbsp;</td>
                <td><input class="camera-checkbox" type="checkbox" name="choose_cameras" value="3">&nbsp;3&nbsp;</td>
              </tr>
              <tr>
                <td><input class="camera-checkbox" type="checkbox" name="choose_cameras" value="4">&nbsp;4&nbsp;</td>
                <td><input class="camera-checkbox" type="checkbox" name="choose_cameras" value="5">&nbsp;5&nbsp;</td>
                <td><input class="camera-checkbox" type="checkbox" name="choose_cameras" value="6">&nbsp;6&nbsp;</td>
                <td><input class="camera-checkbox" type="checkbox" name="choose_cameras" value="7">&nbsp;7&nbsp;</td>
              </tr>
              <tr>
                <td><input class="camera-checkbox" type="checkbox" name="choose_cameras" value="8">&nbsp;8&nbsp;</td>
                <td><input class="camera-checkbox" type="checkbox" name="choose_cameras" value="9">&nbsp;9&nbsp;</td>
                <td><input class="camera-checkbox" type="checkbox" name="choose_cameras" value="10">&nbsp;10&nbsp;</td>
                <td><input class="camera-checkbox" type="checkbox" name="choose_cameras" value="11">&nbsp;11&nbsp;</td>
              </tr>
              <tr>
                <td><input class="camera-checkbox" type="checkbox" name="choose_cameras" value="12">&nbsp;12&nbsp;</td>
                <td><input class="camera-checkbox" type="checkbox" name="choose_cameras" value="13">&nbsp;13&nbsp;</td>
                <td><input class="camera-checkbox" type="checkbox" name="choose_cameras" value="14">&nbsp;14&nbsp;</td>
                <td><input class="camera-checkbox" type="checkbox" name="choose_cameras" value="15">&nbsp;15&nbsp;</td>
              </tr>
              <tr>
                <td><input class="camera-checkbox" type="checkbox" name="choose_cameras" value="16">&nbsp;16&nbsp;</td>
              </tr>
            </table>
          <br>
          <video id="feed_preview" src="http://127.0.0.1:8090/live.webm" width="512" height="512" autoplay></video>
        </div>
      </div>
    </div>
    <div class="row">
      <div class="columns">
        <h4>Latest Video Stats</h4>
        <div id="log_stats_div"></div>
      </div>
    </div>

    <script type="text/javascript">
      var isCapturing = false;
      var isStopped = false;
      var dirOutput = -1;
      var interval = -1;
      var defaultErrorText = "There was an error processing your request";

      function logger(msg) {
        document.getElementById('log_div').innerHTML = msg + "<br><br>";
      }

      function hideLogger() {
        document.getElementById('log_div').innerHTML = '';
      }

      function loggerStats(msg) {
        document.getElementById('log_stats_div').innerHTML = msg + "<br>";
      }

      function showPreviewButtonLayout() {
        $('#loading').show();
        $('#loading_text').text('');
        $('#button_start_preview').hide();
        $('#button_stop_preview').show();
        $('#button_start_record').show();
        $('#button_stop_record').hide();
        $('#button_reset').hide();
        $('#button_update_preview').show();
        $('#table_label').hide();
      }

      function showRecordButtonLayout() {
        $('#button_stop_preview').hide();
        $('#button_start_record').hide();
        $('#button_stop_record').show();
        $('#button_update_preview').hide();
      }

      function resetButtonLayout() {
        $('#loading').hide();
        $('#loading_text').text('');
        $('#button_start_preview').show();
        $('#button_stop_preview').hide();
        $('#button_start_record').hide();
        $('#button_stop_record').hide();
        $('#button_reset').show();
        $('#button_update_preview').hide();
        $('#table_label').show();
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

      function doneCapturing() {
        isCapturing = false;
        isStopped = false;
        logger('Done capturing. Cleaning up...');

        // Check if errors
        var hasErrors = checkErrors();

        if (hasErrors == true) {
          resetButtonLayout();
        } else {
          showStats();

          // Clean logger and loading div
          // Giving it a 2 second pause to make sure everything is back in place
          setTimeout(function(){
            hideLogger();
            resetButtonLayout();
          }, 2000);
        }
      }

      function updateHeartbeat() {
        if (isCapturing) {
          $.ajax({
            url: 'update_heartbeat.php?dir=' + dirOutput,
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

      $(document).ready(function () {
        interval = setInterval(updateHeartbeat, 1000); // update every second

        $('#camera_control').load('camera_control.php');

        var form = $('#form_capture');
        $(form).submit(function(e){
          e.preventDefault();

          // Clear stats
          loggerStats('');

          showPreviewButtonLayout();
          logger('Starting preview...');

          $.ajax({
            url: $(form).attr('action'),
            type: $(form).attr('method'),
            data: $(form).serialize(),
            success: function(response) {
              if (response) {
                if (response === 'ERROR:isAppRunning') {
                  alert('Camera is busy. Please try again.');
                  resetButtonLayout();
                } else {
                  // Camera capture process is not done yet
                  dirOutput = JSON.parse(response)[1];
                  isCapturing = true;
                }
              } else {
                resetButtonLayout();
              }
            },
            error: function() {
              resetButtonLayout();
              alert(defaultErrorText);
            }
          });
        });

        $('#button_stop_preview').click(function() {
          resetButtonLayout();
          logger('Stopping preview...');

          $.ajax({
            url: 'record.php?action=quit',
            type: 'get',
            success: function(response) {
              if (response) {
                if (response == 'OK') {
                  logger('Stopped preview...');
                  doneCapturing();
                } else {
                  logger(response);
                }
              } else {
                alert("Bad response when stopping recording.");
              }
            },
            error: function() {
              alert(defaultErrorText);
            }
          })
        });

        $('#button_start_record').click(function() {
          showRecordButtonLayout();
          logger('Recording...');

          $.ajax({
            url: 'record.php?action=record',
            type: 'get',
            success: function(response) {
              if (response) {
                if (response == 'OK') {
                  logger('Recording...');
                } else {
                  logger(response);
                }
              } else {
                alert("Bad response when initiating recording.");
              }
            },
            error: function() {
              alert(defaultErrorText);
            }
          })
        });

        $('#button_stop_record').click(function() {
          resetButtonLayout();
          showPreviewButtonLayout();
          logger('Stopping...');

          $.ajax({
            url: 'record.php?action=stop',
            type: 'get',
            success: function(response) {
              if (response) {
                if (response == 'OK') {
                  logger('Stopped recording...');
                } else {
                  logger(response);
                }
              } else {
                alert("Bad response when quitting.");
              }
            },
            error: function() {
              alert(defaultErrorText);
            }
          })
        });

        $('#button_reset').click(function(){
          // Reload camera controls
          $('#camera_control').load('camera_control.php');
        });

        var camera_limit = 4;
        $('input.camera-checkbox').on('click', function (evt) {
          if ($('.camera-checkbox:checked').length > camera_limit) {
            this.checked = false;
          }
        });

        $('#button_update_preview').click(function(){
          var shutter = document.getElementById('-shutter_number').value;
          var gain = document.getElementById('-gain_number').value;

          // Get list of cameras used for preview
          var cameras = [];
          $.each($("input[name='choose_cameras']:checked"), function(){
              cameras.push($(this).val());
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
      });
    </script>
  </body>
</html>
