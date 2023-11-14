#!/bin/bash

script_dir=$(dirname "$(realpath "$0")")
$script_dir/package_flight_code.sh

flight_zip=$script_dir/flight.zip

if [ -f $flight_zip ]; then

    echo "Copying $flight_zip to raspberry pi"
    scp $flight_zip pi@raspberrypi.local:~/

    echo "Unzipping flight.zip on raspberry pi"
    ssh pi@raspberrypi.local "unzip ~/flight.zip -d ~/"

else
    echo "$script_dir/flight.zip does not exist"
fi