#!/bin/bash

# Get the directory of the script
script_dir=$(dirname "$(realpath "$0")")
flight_directory="$(realpath "$script_dir/flight")"
output_zip="$(realpath "$script_dir/flight.zip")"

if [ -e $output_zip ]; then
    rm $output_zip
fi

echo "Zipping $flight_directory into $output_zip"

# Use find to locate all files excluding __pycache__ directories and README.md files
cd "$flight_directory" || exit 1
git log --format=format:"%ad %H" --date=iso -1 > pygnc_version.info
find . -type f -not -path "*/__pycache__/*" ! -name "README.md" -exec zip "$output_zip" {} \;
rm pygnc_version.info
cd "$script_dir"
