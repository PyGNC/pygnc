#!/bin/bash

script_dir=$(dirname "$(realpath "$0")")
sense=$script_dir/$1/batch_sensor_gps_data.bin

echo "Copying $sense to raspberry pi"
scp -o UserKnownHostsFile=/dev/null $sense pi@raspberrypi.local:~/sense.bin