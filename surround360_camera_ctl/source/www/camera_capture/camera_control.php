<?php
        $camera_props_file = './camProps_raw';

        if (file_exists($camera_props_file)) {
                $output = file($camera_props_file, FILE_IGNORE_NEW_LINES);
        } else {
                $command = '/usr/local/bin/CameraControl --props --raw --debug';
                $pid = exec($command, $output, $return_var);

                if ($return_var !== 0) { // failure
                        echo 'Something went wrong!';
                        return;
                }
        }

        $prefix = '*PROP*';

        $properties = array();
        foreach ($output as $property) {
                if (strpos($property, $prefix) !== 0) {
                        continue;
                }

                $props = explode(';', $property);
                array_shift($props);

                if ($props[0] === 'whitebalance') {
                        // Split in two
                        $props[] = 1;
                        $prop_red = $prop_blue = $props;
                        $prop_red[0] = 'W.B. (red)';
                        $prop_blue[0] = 'W.B. (blue)';
                        $prop_red[5] .= '1';
                        $prop_blue[5] .= '2';

                        $vals = explode(' ', $props[4]);
                        $prop_red[4] = $vals[0];
                        $prop_blue[4] = $vals[1];

                        $properties[] = $prop_red;
                        $properties[] = $prop_blue;
                        continue;
                }

                $props[] = 0.001;
                $properties[] = $props;
        }

        echo '<table border="0"  id="cssTable">';

        foreach ($properties as $props) {
                $name_id = $props[5];
                $input_params = 'min="'.$props[1].'" max="'.$props[2].'" value="'.$props[4].'" step="'.end($props).'"';
                echo '<tr>';
                echo '<td>'.ucfirst($props[0]).'</td>';
                echo '<td><input type="range" name="'.$name_id.'" id="'.$name_id.'_range" '.$input_params.' oninput="showValue(\''.$name_id.'_number\', this.value)" /></td>';
                echo '<td><input type="number" id="'.$name_id.'_number" style="width: 6em;" '.$input_params.' oninput="showValue(\''.$name_id.'_range\', this.value)" /></td>';
                echo '<td>'.$props[3].'</td>';
                echo '<script type="text/javascript">';
                echo 'function showValue(id, value)';
                echo '{';
                echo '  document.getElementById(id).value=value;';
                echo '}';
                echo '</script>';
                echo '</tr>';
        }
        echo '</table>';
?>
