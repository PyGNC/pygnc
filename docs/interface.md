# PyCubed <-> PyGNC Interface Definition
This document describes the interface between the PyCubed flight software and the PyGNC flight software.

At a hardware level, this interface occurs over a USB/UART TTY.

## Software location
The PyGNC flight software consists of all code contained within the [flight](../flight/) directory of this repository.
It can be packaged into a zip file (with nonessential files removed) by running `./package_flight_code.sh`.

The software is self-contained from a filepath standpoint, so it can be placed in any directory on the Raspberry Pi and operate fine.
The filepaths to data files and log files are fixed, however.

## Batch data file location
PyGNC expects to be provided with two binary data files prior to starting. These files are

* `sense.bin`: contains a history of sensor and GPS measurements, used to do orbit and attitude estimation
* `range.bin`: contains a history of range and GPS measurements, used to do formation estimation

Both of these files are expected to be located at `~/`.

### `sense.bin` file format
The `sense.bin` file consists of five segments of sensor data for every one segment of GPS data (this 5:1 relationship can be modified in the flight software by changing `batch_sensor_gps_packet_num_sensor_packets` in `flight/pygnc/common/constants.py`).
Each segment of sensor data is 30 bytes and consists of the following:

| Name | Bytes | Format |
| ---- | ----- | ------ |
| spacecraft time | 4 | integer spacecraft time since unix epoch |
| mag measurement | 6 | 3 axis raw magnetometer data (as from BMX160 IMU) |
| raw hall | 2 | raw hall measurement (as from BMX160 IMU) unused |
| gyro measurement | 6 | 3 axis raw gyro measurement (as from BMX160 IMU) |
|  sun measurement | 12 | 6 raw sun sensor measurements (as from OPT3001 lux sensor) |


Each segment of GPS data is 99 bytes and consists of the following:
| Name | Bytes | Format |
| ---- | ----- | ------ |
| spacecraft time | 4 | |
| gps time status | 1 | |
| gps week | 2 | |
| gps time of week | 4 | |
| position status | 1 | |
| position_x |8| |
| position_y |8| |
| position_z |8| |
| position_sig_x |4| |
| position_sig_y |4| |
| position_sig_z |4| |
| velocity_status |1| |
| velocity_x |8| |
| velocity_y |8| |
| velocity_z |8| |
| velocity_sig_x |4| |
| velocity_sig_y |4| |
| velocity_sig_z |4| |
| v_latency |4| |
| differential_age |4| |
| solution_age |4| |
| sats_tracked |1| |
| sats_in_solution |1| |

### Range files
Range files are contained in `~/range/`. The file names have the format `rng#####.bin`.

`rng#####.bin` file format
The `rng#####.bin` file is used to communicate range measurements.
It consists of a GPS measurement packet followed by one or more range measurements that occurred at approximately the same time.
Each GPS measurement packet is 102 bytes and starts with `0xaaD`. 
Each Range packet is 12 bytes.

This file is 25.6kB, and covers approximately.

## Starting PyGNC 
To start PyGNC, use the following function call:

```
python pygnc_main.py <timeout_in_minutes> <spacecraft_time_in_seconds>
```

## Logging and Status

Short status with most recent state estimates (less than 1kb)
```
~/pygnc_status.bin
```

Very short human readable status with minimal information (less than 200 bytes)
```
~/pygnc_info.txt
```

### Log files
```
~/pygnc_logs/
```

# Streaming Interface


# Stopping PyGNC
A `CTRL-C` is applied to the main process, the PyCubed controller then attempts to close all of the other processes with a signal. The processes can save any last data and then close. 10 seconds is given between the `CTRL-C` and the Raspberry Pi getting shut down.
